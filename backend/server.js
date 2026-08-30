// SafeWatt phone backend — v28 (Session 27 code-review fixes)
//
// v28 changes (Session 27 — code review):
// - Auth IP check: req.ip.startsWith('192.168.') instead of .includes(), to avoid
//   substring matches against arbitrary header content.
// - Prepared statements lifted to module-level cache (gap-time, daily-stats, dedup-stats,
//   readings range/default/since-id/latest, stats, data-span, relay/anomaly inserts and
//   reads). Cuts statement-prep overhead on every request.
// - /api/data-span: replace MIN/MAX(timestamp) + COUNT(*) full-scan with primary-key
//   lookups via MIN(id)/MAX(id) + per-id timestamp lookup. ~5ms instead of ~50–100ms
//   on cache miss, important because the dashboard hits this every 30s.
// - CORS: whitelisted to LAN dashboard origin only. Add Serveo origin here when tunnel
//   returns from deferral. ESP32 sends no Origin header so it is unaffected.
// - Error responses: log err.message internally, return generic { error: 'internal' }
//   externally. Prevents SQL/path detail leakage if the tunnel is ever re-enabled.
// - /api/relay/pending: marks stuck commands older than 5 minutes as 'expired' before
//   selecting next pending. Prevents a single stuck command from blocking the queue.
// - /api/anomalies POST: insert BEFORE responding (was: respond first, log silently
//   on failure). Q3.1 anomaly detection accuracy depends on every detection being
//   logged; firing-and-forgetting hides insert failures from the firmware.
//
// Carried forward from v27:
// - /api/readings/latest appliance/id index + 2s TTL cache.
// - gap_excess_seconds in /api/gap-time and /api/daily-stats.
//
// Carried forward from v26:
// - GET /api/dedup-stats?from&to server-side dedup aggregation.
//
// Carried forward from v25:
// - Cache key quantization (30s `to` buckets).
// - /api/readings?from&to LIMIT 5000 + X-Truncated header.
//
// Carried forward from v24/v23/v22:
// - 10s TTL response cache for gap-time, daily-stats, stats, data-span.
// - Chunked id-paginated iteration in /api/gap-time and /api/daily-stats.
// - Per-request logger + slow-query marker + [CACHE HIT] log marker.
// - LIMIT caps on /api/anomalies and /api/readings/since-id.
// - Session 22's mutable INSERT, supervisor exit, lock cleanup.

const express = require('express');
const { Database } = require('node-sqlite3-wasm');
const cors = require('cors');
const path = require('path');
const fs = require('fs');

const app = express();

// v28: CORS whitelist. Only the LAN dashboard origin is allowed for cross-origin reads.
// ESP32 (no Origin header) is unaffected; same-origin dashboard fetches don't need CORS.
const ALLOWED_ORIGINS = ['http://192.168.1.58:3000'];
app.use(cors({
  origin: function(origin, cb) {
    if (!origin) return cb(null, true); // ESP32 / curl
    if (ALLOWED_ORIGINS.indexOf(origin) !== -1) return cb(null, true);
    cb(new Error('Origin not allowed: ' + origin));
  }
}));

app.use(express.json());

// Session 24: lightweight per-request log + slow-query marker.
app.use(function(req, res, next) {
  if (req.path === '/' || req.path.startsWith('/index')) return next();
  const start = Date.now();
  const tag = req.method + ' ' + req.originalUrl;
  console.log('[REQ ' + new Date().toISOString() + '] ' + tag);
  res.on('finish', function() {
    const ms = Date.now() - start;
    if (ms > 500) console.log('[SLOW ' + ms + 'ms] ' + tag + ' -> ' + res.statusCode);
  });
  next();
});

// Session 24 (v24+v25): short-TTL response cache for read-heavy endpoints.
const CACHE_TTL_MS = 10000;
const LATEST_CACHE_TTL_MS = 2000;
const QUANTIZE_MS = 30000;
const ESP32_INTERVAL_S = 7;
const responseCache = new Map();

function cacheKey(req) {
  const idx = req.originalUrl.indexOf('?');
  if (idx === -1) return req.originalUrl;
  const path = req.originalUrl.slice(0, idx);
  const params = new URLSearchParams(req.originalUrl.slice(idx + 1));
  const to = params.get('to');
  if (to) {
    const t = Date.parse(to);
    if (!isNaN(t)) {
      const quantized = new Date(Math.floor(t / QUANTIZE_MS) * QUANTIZE_MS);
      params.set('to', quantized.toISOString().slice(0, 19));
    }
  }
  return path + '?' + params.toString();
}

function withCache(req, fn, ttlMs) {
  const key = cacheKey(req);
  const now = Date.now();
  const hit = responseCache.get(key);
  if (hit && hit.expires > now) {
    console.log('[CACHE HIT] ' + key);
    return hit.value;
  }
  const value = fn();
  responseCache.set(key, { value: value, expires: now + (ttlMs || CACHE_TTL_MS) });
  if (responseCache.size > 100) {
    for (const [k, v] of responseCache) {
      if (v.expires <= now) responseCache.delete(k);
    }
  }
  return value;
}

// v28: generic 500 helper. Internal error stays in the log; client sees only 'internal'.
function send500(res, label, err) {
  console.error(label + ' error:', err && err.message);
  return res.status(500).json({ error: 'internal' });
}

// Session 22: clear stale dotlock from prior crash so DB can open cleanly
const LOCK_DIR = process.env.HOME + '/safewatt.db.lock';
if (fs.existsSync(LOCK_DIR)) {
  try { fs.rmSync(LOCK_DIR, { recursive: true, force: true }); console.log('Cleared stale DB lock at startup'); }
  catch (err) { console.warn('Could not clear stale lock:', err.message); }
}

const db = new Database(process.env.HOME + '/safewatt.db');

db.exec("PRAGMA journal_mode = WAL; PRAGMA synchronous = NORMAL; PRAGMA wal_autocheckpoint = 1000;");
db.exec("CREATE INDEX IF NOT EXISTS idx_readings_timestamp ON readings(timestamp);");
db.exec("CREATE INDEX IF NOT EXISTS idx_readings_channel_id ON readings(channel, id);");
db.exec("CREATE INDEX IF NOT EXISTS idx_readings_appliance_id ON readings(appliance, id);");
db.exec(
  'CREATE TABLE IF NOT EXISTS readings (' +
  'id INTEGER PRIMARY KEY AUTOINCREMENT,' +
  "timestamp TEXT DEFAULT (strftime('%Y-%m-%dT%H:%M:%S', 'now', 'localtime'))," +
  'channel INTEGER NOT NULL,' +
  'appliance TEXT,' +
  'voltage REAL,' +
  'current_a REAL,' +
  'power_w REAL,' +
  'energy_kwh REAL,' +
  'cost_php REAL,' +
  'carbon_kg REAL,' +
  'uptime_ms INTEGER);' +
  'CREATE TABLE IF NOT EXISTS anomalies (' +
  'id INTEGER PRIMARY KEY AUTOINCREMENT,' +
  "timestamp TEXT DEFAULT (strftime('%Y-%m-%dT%H:%M:%S', 'now', 'localtime'))," +
  'channel INTEGER NOT NULL,' +
  'appliance TEXT,' +
  'detected_wattage REAL,' +
  'threshold_wattage REAL,' +
  'deviation_pct REAL,' +
  'response_time_ms INTEGER,' +
  'action_taken TEXT);' +
  'CREATE TABLE IF NOT EXISTS relay_commands (' +
  'id INTEGER PRIMARY KEY AUTOINCREMENT,' +
  "timestamp TEXT DEFAULT (strftime('%Y-%m-%dT%H:%M:%S', 'now', 'localtime'))," +
  'channel INTEGER NOT NULL,' +
  'action TEXT NOT NULL,' +
  "status TEXT DEFAULT 'pending');" +
  // v29 (Session 32): Layer 2 adaptive threshold storage.
  // thresholds  = the current learned threshold per channel, fetched by the ESP32 on boot.
  // threshold_training_log = every running-max sample recorded during a training window,
  //   which is what Q3.4 (convergence behaviour) is analysed from.
  'CREATE TABLE IF NOT EXISTS thresholds (' +
  'channel INTEGER PRIMARY KEY,' +
  'appliance TEXT,' +
  'learned_max_w REAL NOT NULL,' +
  'threshold_w REAL NOT NULL,' +
  'multiplier REAL NOT NULL,' +
  'samples INTEGER,' +
  'training_seconds INTEGER,' +
  "trained_at TEXT DEFAULT (strftime('%Y-%m-%dT%H:%M:%S', 'now', 'localtime')));" +
  'CREATE TABLE IF NOT EXISTS threshold_training_log (' +
  'id INTEGER PRIMARY KEY AUTOINCREMENT,' +
  "timestamp TEXT DEFAULT (strftime('%Y-%m-%dT%H:%M:%S', 'now', 'localtime'))," +
  'channel INTEGER NOT NULL,' +
  'elapsed_s INTEGER,' +
  'sample_n INTEGER,' +
  'power_w REAL,' +
  'running_max_w REAL);'
);

// Session 22: mutable prepared statement with re-prepare recovery + exit-after-N-failures
let insertReading;
function prepareInsertStmt() {
  insertReading = db.prepare(
    'INSERT INTO readings (channel, appliance, voltage, current_a, power_w, energy_kwh, cost_php, carbon_kg, uptime_ms)' +
    ' VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)'
  );
}
prepareInsertStmt();
let insertFailureStreak = 0;
const INSERT_FAILURE_EXIT_THRESHOLD = 5;

// v28: lift all read prepared statements to module-level. Saves the parser+planner cost
// on every request. They're fixed SQL with bound parameters; only cached SELECTs benefit.
const stmts = {
  readingsByRange: db.prepare(
    'SELECT * FROM readings WHERE timestamp >= ? AND timestamp < ? ORDER BY id DESC LIMIT ?'
  ),
  readingsDefault: db.prepare(
    'SELECT * FROM readings ORDER BY id DESC LIMIT 500'
  ),
  readingsSinceId: db.prepare(
    'SELECT * FROM readings WHERE id > ? ORDER BY id DESC LIMIT 5000'
  ),
  readingsLatest: db.prepare(
    'SELECT r1.* FROM readings r1' +
    ' INNER JOIN (SELECT appliance, MAX(id) AS max_id FROM readings WHERE appliance IS NOT NULL GROUP BY appliance) r2' +
    ' ON r1.appliance = r2.appliance AND r1.id = r2.max_id'
  ),
  gapTimeChunk: db.prepare(
    'SELECT id, timestamp FROM readings' +
    ' WHERE channel = 1 AND id > ? AND timestamp >= ? AND timestamp < ?' +
    ' ORDER BY id LIMIT ?'
  ),
  dailyStatsCounts: db.prepare(
    "SELECT date(timestamp) AS day, COUNT(*) AS count" +
    " FROM readings WHERE timestamp >= ? AND timestamp < ?" +
    " GROUP BY date(timestamp)"
  ),
  dailyStatsChunk: db.prepare(
    'SELECT id, timestamp FROM readings' +
    ' WHERE channel = 1 AND id > ? AND timestamp >= ? AND timestamp < ?' +
    ' ORDER BY id LIMIT ?'
  ),
  statsByRange: db.prepare(
    'SELECT appliance, COUNT(*) AS count FROM readings WHERE timestamp >= ? AND timestamp < ? GROUP BY appliance'
  ),
  statsAll: db.prepare(
    'SELECT appliance, COUNT(*) AS count FROM readings GROUP BY appliance'
  ),
  // v28: indexed primary-key lookups instead of MIN/MAX(timestamp) + COUNT(*) full-scan.
  dataSpanMinId: db.prepare('SELECT MIN(id) AS id FROM readings'),
  dataSpanMaxId: db.prepare('SELECT MAX(id) AS id FROM readings'),
  readingTsById: db.prepare('SELECT timestamp FROM readings WHERE id = ?'),
  dedupOverall: db.prepare(
    'SELECT COUNT(*) AS total_rows,' +
    ' COUNT(DISTINCT timestamp || channel) AS unique_rows,' +
    ' COUNT(DISTINCT timestamp) AS unique_cycles' +
    ' FROM readings WHERE timestamp >= ? AND timestamp < ?'
  ),
  dedupPerChannel: db.prepare(
    'SELECT channel, COUNT(*) AS rows, COUNT(DISTINCT timestamp) AS unique_cycles' +
    ' FROM readings WHERE timestamp >= ? AND timestamp < ?' +
    ' GROUP BY channel ORDER BY channel'
  ),
  insertAnomaly: db.prepare(
    'INSERT INTO anomalies (channel, appliance, detected_wattage, threshold_wattage, deviation_pct, response_time_ms, action_taken)' +
    ' VALUES (?, ?, ?, ?, ?, ?, ?)'
  ),
  selectAnomalies: db.prepare(
    'SELECT * FROM anomalies ORDER BY timestamp DESC LIMIT 1000'
  ),
  insertRelayCommand: db.prepare(
    "INSERT INTO relay_commands (channel, action, status) VALUES (?, ?, 'pending')"
  ),
  // v28: expire commands older than 5 minutes so a stuck command does not block newer ones.
  expireStaleRelay: db.prepare(
    "UPDATE relay_commands SET status = 'expired'" +
    " WHERE status = 'pending' AND timestamp < datetime('now', 'localtime', '-5 minutes')"
  ),
  selectRelayPending: db.prepare(
    "SELECT * FROM relay_commands WHERE status = 'pending' ORDER BY id ASC LIMIT 1"
  ),
  upsertThreshold: db.prepare(
    'INSERT INTO thresholds (channel, appliance, learned_max_w, threshold_w, multiplier, samples, training_seconds)' +
    ' VALUES (?, ?, ?, ?, ?, ?, ?)' +
    ' ON CONFLICT(channel) DO UPDATE SET appliance=excluded.appliance, learned_max_w=excluded.learned_max_w,' +
    ' threshold_w=excluded.threshold_w, multiplier=excluded.multiplier, samples=excluded.samples,' +
    " training_seconds=excluded.training_seconds, trained_at=strftime('%Y-%m-%dT%H:%M:%S','now','localtime')"
  ),
  selectThreshold: db.prepare('SELECT * FROM thresholds WHERE channel = ?'),
  selectAllThresholds: db.prepare('SELECT * FROM thresholds ORDER BY channel'),
  insertTrainingSample: db.prepare(
    'INSERT INTO threshold_training_log (channel, elapsed_s, sample_n, power_w, running_max_w) VALUES (?, ?, ?, ?, ?)'
  ),
  selectTrainingLog: db.prepare(
    'SELECT * FROM threshold_training_log WHERE channel = ? ORDER BY id LIMIT 5000'
  ),
  lastTripForChannel: db.prepare('SELECT timestamp FROM anomalies WHERE channel = ? ORDER BY id DESC LIMIT 1'),
  lastCmdForChannel: db.prepare("SELECT timestamp, action, status FROM relay_commands WHERE channel = ? AND status IN ('pending','executed') ORDER BY id DESC LIMIT 1"),
  ackRelay: db.prepare(
    "UPDATE relay_commands SET status = 'executed' WHERE id = ?"
  )
};

console.log('SQLite database ready at ' + process.env.HOME + '/safewatt.db');

// v28: startsWith instead of includes to avoid matching arbitrary substrings.
const requireAuth = function(req, res, next) {
  const ip = req.ip || req.connection.remoteAddress || '';
  const stripped = ip.replace(/^::ffff:/, '');
  if (stripped.startsWith('192.168.') || stripped === '127.0.0.1' || stripped === '::1') return next();
  const authHeader = req.headers['authorization'];
  const token = authHeader && authHeader.startsWith('Bearer ') && authHeader.slice(7);
  if (!token || token !== process.env.API_KEY) return res.status(401).json({ error: 'Unauthorized' });
  next();
};

app.get('/', function(req, res) {
  res.sendFile(path.join(__dirname, 'index.html'));
});

app.post('/api/readings', requireAuth, function(req, res) {
  const t0 = Date.now();
  const rows = Array.isArray(req.body) ? req.body : [req.body];
  function runOne(row) {
    insertReading.run([row.channel, row.appliance, row.voltage, row.current_a, row.power_w, row.energy_kwh, row.cost_php, row.carbon_kg, row.uptime_ms || null]);
  }
  try {
    for (let i = 0; i < rows.length; i++) {
      try {
        runOne(rows[i]);
        insertFailureStreak = 0;
      } catch (innerErr) {
        console.warn('INSERT failed (' + innerErr.message + '); re-preparing statement and retrying');
        try { prepareInsertStmt(); runOne(rows[i]); insertFailureStreak = 0; }
        catch (retryErr) {
          insertFailureStreak++;
          if (insertFailureStreak >= INSERT_FAILURE_EXIT_THRESHOLD) {
            console.error('Repeated INSERT failures (' + insertFailureStreak + ') -- exiting for supervisor restart');
            process.exit(1);
          }
          throw retryErr;
        }
      }
    }
    const dt = Date.now() - t0;
    if (dt > 200) console.log('[SLOW INSERT] ' + dt + 'ms for ' + rows.length + ' rows');
    res.json({ status: 'ok', inserted: rows.length });
  } catch (err) {
    send500(res, 'POST /api/readings', err);
  }
});

app.get('/api/readings', function(req, res) {
  try {
    const fromDate = req.query.from;
    const toDate = req.query.to;
    const READINGS_LIMIT = 5000;
    let rows;
    if (fromDate && toDate) {
      rows = stmts.readingsByRange.all([fromDate, toDate, READINGS_LIMIT]);
    } else {
      rows = stmts.readingsDefault.all();
    }
    if (rows.length === READINGS_LIMIT && fromDate && toDate) {
      res.set('X-Truncated', 'true');
      res.set('X-Limit', String(READINGS_LIMIT));
    }
    res.json(rows);
  } catch (err) {
    send500(res, 'GET /api/readings', err);
  }
});

const CHUNK_SIZE = 5000;

app.get('/api/gap-time', function(req, res) {
  const fromDate = req.query.from;
  const toDate = req.query.to;
  if (!fromDate || !toDate) return res.status(400).json({ error: 'from and to required' });
  try {
    const result = withCache(req, function() {
      let prev_ts = null;
      let window_first_ts = null;
      let gap_seconds = 0;
      let gap_excess_seconds = 0;
      let gap_count = 0;
      let last_id = 0;
      while (true) {
        const rows = stmts.gapTimeChunk.all([last_id, fromDate, toDate, CHUNK_SIZE]);
        if (rows.length === 0) break;
        for (let i = 0; i < rows.length; i++) {
          const ts = rows[i].timestamp;
          if (window_first_ts === null) window_first_ts = ts;
          if (prev_ts !== null) {
            const gap = (Date.parse(ts) - Date.parse(prev_ts)) / 1000;
            if (gap > 10) {
              gap_seconds += gap;
              gap_excess_seconds += Math.max(0, gap - ESP32_INTERVAL_S);
              gap_count++;
            }
          }
          prev_ts = ts;
        }
        last_id = rows[rows.length - 1].id;
        if (rows.length < CHUNK_SIZE) break;
      }
      return {
        gap_seconds: gap_seconds,
        gap_excess_seconds: gap_excess_seconds,
        expected_interval_s: ESP32_INTERVAL_S,
        gap_count: gap_count,
        window_first_ts: window_first_ts
      };
    });
    res.json(result);
  } catch (err) {
    send500(res, 'GET /api/gap-time', err);
  }
});

app.get('/api/daily-stats', function(req, res) {
  const fromDate = req.query.from;
  const toDate = req.query.to;
  if (!fromDate || !toDate) return res.status(400).json({ error: 'from and to required' });
  try {
    const result = withCache(req, function() {
      const counts = stmts.dailyStatsCounts.all([fromDate, toDate]);
      const out = {};
      counts.forEach(function(r) {
        out[r.day] = { count: r.count, gap_seconds: 0, gap_excess_seconds: 0, gap_count: 0, first_ts: null, last_ts: null };
      });
      let prev_ts = null;
      let prev_day = null;
      let last_id = 0;
      while (true) {
        const rows = stmts.dailyStatsChunk.all([last_id, fromDate, toDate, CHUNK_SIZE]);
        if (rows.length === 0) break;
        for (let i = 0; i < rows.length; i++) {
          const ts = rows[i].timestamp;
          const day = ts.substring(0, 10);
          if (!out[day]) out[day] = { count: 0, gap_seconds: 0, gap_excess_seconds: 0, gap_count: 0, first_ts: null, last_ts: null };
          if (out[day].first_ts === null) out[day].first_ts = ts;
          out[day].last_ts = ts;
          if (prev_ts !== null && prev_day === day) {
            const gap = (Date.parse(ts) - Date.parse(prev_ts)) / 1000;
            if (gap > 10) {
              out[day].gap_seconds += gap;
              out[day].gap_excess_seconds += Math.max(0, gap - ESP32_INTERVAL_S);
              out[day].gap_count++;
            }
          }
          prev_ts = ts;
          prev_day = day;
        }
        last_id = rows[rows.length - 1].id;
        if (rows.length < CHUNK_SIZE) break;
      }
      return out;
    });
    res.json(result);
  } catch (err) {
    send500(res, 'GET /api/daily-stats', err);
  }
});

app.get('/api/stats', function(req, res) {
  const fromDate = req.query.from;
  const toDate = req.query.to;
  try {
    const stats = withCache(req, function() {
      const result = (fromDate && toDate)
        ? stmts.statsByRange.all([fromDate, toDate])
        : stmts.statsAll.all();
      const out = { rice_cooker: 0, tv: 0, phone_charger: 0, total: 0 };
      result.forEach(r => {
        if (r.appliance && out[r.appliance] !== undefined) { out[r.appliance] = r.count; out.total += r.count; }
      });
      return out;
    });
    res.json(stats);
  } catch (err) {
    send500(res, 'GET /api/stats', err);
  }
});

// v28: indexed primary-key lookup. MIN/MAX(id) hits the AUTOINCREMENT index in O(log n).
// total_rows is approximated as max_id - min_id + 1, accurate as long as no rows are
// deleted (research data is append-only).
app.get('/api/data-span', function(req, res) {
  try {
    const result = withCache(req, function() {
      const minRow = stmts.dataSpanMinId.get();
      const maxRow = stmts.dataSpanMaxId.get();
      const minId = minRow ? minRow.id : null;
      const maxId = maxRow ? maxRow.id : null;
      if (minId === null || maxId === null) {
        return { first_ts: null, last_ts: null, total_rows: 0 };
      }
      const firstRow = stmts.readingTsById.get([minId]);
      const lastRow = stmts.readingTsById.get([maxId]);
      return {
        first_ts: firstRow ? firstRow.timestamp : null,
        last_ts: lastRow ? lastRow.timestamp : null,
        total_rows: maxId - minId + 1
      };
    });
    res.json(result);
  } catch (err) {
    send500(res, 'GET /api/data-span', err);
  }
});

app.get('/api/dedup-stats', function(req, res) {
  const fromDate = req.query.from;
  const toDate = req.query.to;
  if (!fromDate || !toDate) return res.status(400).json({ error: 'from and to required' });
  try {
    const result = withCache(req, function() {
      const overall = stmts.dedupOverall.get([fromDate, toDate]);
      const perChannel = stmts.dedupPerChannel.all([fromDate, toDate]);
      const duplicates = overall.total_rows - overall.unique_rows;
      const dup_rate_pct = overall.total_rows > 0
        ? Math.round((duplicates / overall.total_rows) * 10000) / 100
        : 0;
      return {
        total_rows: overall.total_rows,
        unique_rows: overall.unique_rows,
        unique_cycles: overall.unique_cycles,
        duplicates: duplicates,
        dup_rate_pct: dup_rate_pct,
        per_channel: perChannel
      };
    });
    res.json(result);
  } catch (err) {
    send500(res, 'GET /api/dedup-stats', err);
  }
});

app.get('/api/readings/latest', function(req, res) {
  try {
    const rows = withCache({ originalUrl: '/api/readings/latest' }, function() {
      return stmts.readingsLatest.all();
    }, LATEST_CACHE_TTL_MS);
    res.json(rows);
  } catch (err) {
    send500(res, 'GET /api/readings/latest', err);
  }
});

app.get('/api/readings/since-id/:id', function(req, res) {
  const id = parseInt(req.params.id, 10);
  if (isNaN(id)) return res.status(400).json({ error: 'invalid id' });
  try {
    const rows = stmts.readingsSinceId.all([id]);
    res.json(rows);
  } catch (err) {
    send500(res, 'GET /api/readings/since-id', err);
  }
});

// v28: insert BEFORE responding so a failed insert returns 500 instead of being
// silently swallowed. Q3.1 anomaly detection accuracy depends on every detected
// anomaly being persisted; firing-and-forgetting masks DB-side failures from the firmware.
app.post('/api/anomalies', requireAuth, function(req, res) {
  const b = req.body;
  try {
    stmts.insertAnomaly.run([b.channel, b.appliance, b.detected_wattage, b.threshold_wattage, b.deviation_pct, b.response_time_ms, b.action_taken]);
    res.json({ status: 'ok' });
  } catch (err) {
    send500(res, 'POST /api/anomalies', err);
  }
});

app.get('/api/anomalies', function(req, res) {
  try {
    const rows = stmts.selectAnomalies.all();
    res.json(rows);
  } catch (err) {
    send500(res, 'GET /api/anomalies', err);
  }
});

app.post('/api/relay', requireAuth, function(req, res) {
  const channel = req.body.channel;
  const action = req.body.action;
  if (!channel || (action !== 'on' && action !== 'off')) return res.status(400).json({ error: 'channel and action (on|off) required' });
  try {
    stmts.insertRelayCommand.run([channel, action]);
    res.json({ status: 'ok', message: 'Relay command queued: ch' + channel + ' ' + action });
  } catch (err) { send500(res, 'POST /api/relay', err); }
});

// v28: expire stale pending commands (>5 minutes old) before returning the next pending.
// Prevents one stuck command from permanently blocking newer queue entries.
app.get('/api/relay/pending', function(req, res) {
  try {
    stmts.expireStaleRelay.run();
    const row = stmts.selectRelayPending.get();
    res.json(row || null);
  } catch (err) { send500(res, 'GET /api/relay/pending', err); }
});

app.post('/api/relay/ack', requireAuth, function(req, res) {
  const id = req.body.id;
  if (!id) return res.status(400).json({ error: 'id required' });
  try {
    stmts.ackRelay.run([id]);
    res.json({ status: 'ok', acknowledged: id });
  } catch (err) { send500(res, 'POST /api/relay/ack', err); }
});


// ============================================================
// v29 (Session 32) — LAYER 2 ADAPTIVE THRESHOLD
// The ESP32 learns each appliance's maximum wattage over a training window, multiplies it
// by LAYER2_MULTIPLIER, and persists the result here so the threshold survives a reboot
// (Q3.5). Every running-max sample taken during training is also logged, which is the data
// Q3.4 (convergence behaviour) is analysed from.
// ============================================================

// Store / replace a learned threshold. Called by the ESP32 at the end of a training window.
app.post('/api/threshold', requireAuth, function(req, res) {
  const b = req.body || {};
  if (b.channel === undefined || b.threshold_w === undefined) {
    return res.status(400).json({ error: 'channel and threshold_w required' });
  }
  try {
    stmts.upsertThreshold.run([
      b.channel, b.appliance || null, b.learned_max_w, b.threshold_w,
      b.multiplier, b.samples || null, b.training_seconds || null
    ]);
    console.log('[THRESHOLD] ch' + b.channel + ' learned_max=' + b.learned_max_w +
                'W threshold=' + b.threshold_w + 'W samples=' + b.samples);
    res.json({ status: 'ok' });
  } catch (err) { send500(res, 'POST /api/threshold', err); }
});

// Fetch one channel's threshold. The ESP32 calls this on boot before falling back to NVS.
app.get('/api/threshold/:channel', function(req, res) {
  try {
    const row = stmts.selectThreshold.get([parseInt(req.params.channel, 10)]);
    res.json(row || null);
  } catch (err) { send500(res, 'GET /api/threshold/:channel', err); }
});

// All thresholds — used by the dashboard Status tab.
app.get('/api/thresholds', function(req, res) {
  try { res.json(stmts.selectAllThresholds.all()); }
  catch (err) { send500(res, 'GET /api/thresholds', err); }
});

// Append one convergence sample during a training window (Q3.4 source data).
app.post('/api/threshold/training', requireAuth, function(req, res) {
  const b = req.body || {};
  if (b.channel === undefined) return res.status(400).json({ error: 'channel required' });
  try {
    stmts.insertTrainingSample.run([b.channel, b.elapsed_s, b.sample_n, b.power_w, b.running_max_w]);
    res.json({ status: 'ok' });
  } catch (err) { send500(res, 'POST /api/threshold/training', err); }
});

// Retrieve a channel's full training curve for convergence analysis.
app.get('/api/threshold/training/:channel', function(req, res) {
  try { res.json(stmts.selectTrainingLog.all([parseInt(req.params.channel, 10)])); }
  catch (err) { send500(res, 'GET /api/threshold/training/:channel', err); }
});

// v30 (Session 32): derived relay state per channel.
// The dashboard toggle previously tracked only its own clicks, so an automatic over-wattage
// trip left the toggle showing ON while the relay was actually OPEN — and the two toggles
// (appliance card + manual control panel) could disagree with each other.
// There is no direct relay-state field in the readings, so state is derived from whichever
// happened most recently per channel: an anomaly trip (always turns the relay OFF) or an
// executed manual relay command.
app.get('/api/relay/state', function(req, res) {
  try {
    const out = {};
    for (var ch = 1; ch <= 3; ch++) {
      const lastTrip = stmts.lastTripForChannel.get([ch]);
      const lastCmd  = stmts.lastCmdForChannel.get([ch]);
      var state = 'on';
      var src   = 'default';
      var tripT = lastTrip ? lastTrip.timestamp : null;
      var cmdT  = lastCmd  ? lastCmd.timestamp  : null;
      if (tripT && (!cmdT || tripT >= cmdT)) { state = 'off'; src = 'anomaly_trip'; }
      else if (cmdT) { state = lastCmd.action; src = 'manual_command'; }
      out['ch' + ch] = { state: state, source: src, trip_at: tripT, cmd_at: cmdT };
    }
    res.json(out);
  } catch (err) { send500(res, 'GET /api/relay/state', err); }
});
const PORT = process.env.PORT || 3000;
app.listen(PORT, '0.0.0.0', function() {
  console.log('SafeWatt phone backend running on port ' + PORT);
  console.log('Local: http://192.168.1.58:' + PORT);
  console.log('Auth: API_KEY env var required for write/relay endpoints');
});
