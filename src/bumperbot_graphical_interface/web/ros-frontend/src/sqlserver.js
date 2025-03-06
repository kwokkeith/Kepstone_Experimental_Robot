// sqlite3 sqlserver.js

const sqlite3 = require('sqlite3').verbose();
const express = require('express');
const path = require('path');
const cors = require('cors');

const app = express();
const PORT = 5000;

// MIDDLEWARE
app.use(cors());
app.use(express.json());

// CONNECT TO SQLITE3 DATABASE
const dbPath = path.resolve(__dirname, '..', '..', '..', 'db', 'coverage_zones.db');
const db = new sqlite3.Database(dbPath, (err) => {
    if (err) {
        console.log(dbPath);
        console.error(err.message);
    }
    else {
        console.log('Connected to the coverage_zones database.');
    }
}
);

// API ENDPOINTS

//get bcdpolycontourdata from merged table
app.get('/api/bcdpolycontourdata', (req, res) => {
    const query = `
        SELECT c1.map_name, c2.bcdPolygonContour_coordinates, c2.polygon_index
        FROM Config_Table c1 
        INNER JOIN Polygon_Table c2 
        ON c1.config_id = c2.config_id
        ORDER BY c2.polygon_id ASC
    `;
    db.all(query, [], (err, rows) => {
        if (err) {
            res.status(500).json({ error: err.message });
            return;
        }
        res.json({ data: rows });
    });
});

// get cleaning_path_coordinates from config_table
app.get('/api/cleaning_waypoints', (req, res) => {
    const query = `SELECT map_name, cleaning_path_coordinates FROM Config_Table`;
    db.all(query, [], (err, rows) => {
      if (err) {
        res.status(500).json({ error: err.message });
        return;
      }
      res.json({ data: rows });
    });
});

// get data from config_table
app.get('/api/config', (req, res) => {
    const query = `SELECT * FROM Config_Table`;
    db.all(query, [], (err, rows) => {
      if (err) {
        res.status(500).json({ error: err.message });
        return;
      }
      res.json({ data: rows });
    });
});

// get data from polygon table
app.get('/api/polygon', (req, res) => {
    const query = `SELECT * FROM Polygon_Table`;
    db.all(query, [], (err, rows) => {
      if (err) {
        res.status(500).json({ error: err.message });
        return;
      }
      res.json({ data: rows });
    });
});

// DELETE map by mapName in map_name column
app.delete('/api/maps/:mapName', (req, res) => {
    const { mapName } = req.params;
    
    // First, check if a record exists so we can return 404 early if not found.
    const sqlExists = `SELECT COUNT(*) AS count FROM Config_Table WHERE map_name = ?`;
    db.get(sqlExists, [mapName], (err, row) => {
      if (err) {
        console.error('Existence check error:', err);
        return res.status(500).json({ error: 'Database error' });
      }
      if (row.count === 0) {
        return res.status(404).json({ message: 'No record found to delete.' });
      }
      
      db.serialize(() => {
        db.run('BEGIN TRANSACTION', (err) => {
          if (err) {
            console.error('Error starting transaction:', err);
            return res.status(500).json({ error: 'Error starting transaction' });
          }
    
          const sqlDeletePolygon = `DELETE FROM Polygon_Table
                  WHERE config_id IN (SELECT config_id FROM Config_Table WHERE map_name = ?)`;
    
          db.run(sqlDeletePolygon, [mapName], function(err) {
            if (err) {
              console.error('Error deleting Polygon records:', err);
              return db.run('ROLLBACK', () => {
                res.status(500).json({ error: 'Error deleting Polygon records' });
              });
            }
    
            const sqlDeleteConfig = `DELETE FROM Config_Table WHERE map_name = ?`;
    
            db.run(sqlDeleteConfig, [mapName], function(err) {
              if (err) {
                console.error('Error deleting Config records:', err);
                return db.run('ROLLBACK', () => {
                  res.status(500).json({ error: 'Error deleting Config records' });
                });
              }
    
              db.run('COMMIT', function(err) {
                if (err) {
                  console.error('Error committing transaction:', err);
                  return res.status(500).json({ error: 'Error committing transaction' });
                }
                res.json({ message: `Map with name ${mapName} deleted`, changes: this.changes });
              });
            });
          });
        });
      });
    });
  });

//SQL_INSERT_INTO_SCHEDULES
// Insert data into Jobs_Table
app.post('/api/jobs', (req, res) => {
    const { job_id, config_id, sequence_no } = req.body;
    const query = `INSERT INTO Jobs_Table (job_id, config_id, sequence_no) VALUES (?, ?, ?)`;
    db.run(query, [job_id, config_id, sequence_no], function(err) {
        if (err) {
            return res.status(500).json({ error: err.message });
        }
        res.json({ message: 'Job inserted successfully', job_id: this.lastID });
    });
});

// Insert data into Job_Order_Table
app.post('/api/job_order', (req, res) => {
    const { job_id, config_id } = req.body;
    const query = `INSERT INTO Job_Order_Table (job_id, config_id) VALUES (?, ?)`;
    db.run(query, [job_id, config_id], function(err) {
        if (err) {
            return res.status(500).json({ error: err.message });
        }
        res.json({ message: 'Job order inserted successfully' });
    });
});

// Insert data into Schedule_Table
app.post('/api/schedule', (req, res) => {
    const { schedule_id, schedule_name, start_time, start_date, recurrence } = req.body;
    const query = `INSERT INTO Schedule_Table (schedule_id, schedule_name, start_time, start_date, recurrence) VALUES (?, ?, ?, ?, ?)`;
    db.run(query, [schedule_id, schedule_name, start_time, start_date, recurrence], function(err) {
        if (err) {
            console.error('Error inserting into Schedule_Table:', err.message);
            return res.status(500).json({ error: err.message });
        }
        res.json({ message: 'Schedule inserted successfully', schedule_id: this.lastID });
    });
});

// Insert data into Schedule_Job_Link
app.post('/api/schedule_job_link', (req, res) => {
    const { schedule_id, sequence_no, job_id } = req.body;
    const query = `INSERT INTO Schedule_Job_Link (schedule_id, sequence_no, job_id) VALUES (?, ?, ?)`;
    db.run(query, [schedule_id, sequence_no, job_id], function(err) {
        if (err) {
            console.error('Error inserting into Schedule_Job_Link:', err.message);
            return res.status(500).json({ error: err.message });
        }
        res.json({ message: 'Schedule-Job link inserted successfully' });
    });
});

// Insert data into JobStatus
app.post('/api/job_status', (req, res) => {
    const { job_id, status } = req.body;
    const query = `INSERT INTO JobStatus (job_id, status) VALUES (?, ?)`;
    db.run(query, [job_id, status], function(err) {
        if (err) {
            console.error('Error inserting into JobStatus:', err.message);
            return res.status(500).json({ error: err.message });
        }
        res.json({ message: 'Job status inserted successfully' });
    });
});

// Get config_id for each map_name in zoneSequence
app.post('/api/config_ids', (req, res) => {
    const { zoneSequence } = req.body;
    if (!Array.isArray(zoneSequence) || zoneSequence.length === 0) {
        return res.status(400).json({ error: 'Invalid zoneSequence' });
    }

    const placeholders = zoneSequence.map(() => '?').join(',');
    const query = `SELECT map_name, config_id FROM Config_Table WHERE map_name IN (${placeholders})`;

    db.all(query, zoneSequence, (err, rows) => {
        if (err) {
            console.error('Error fetching config IDs:', err.message);
            return res.status(500).json({ error: err.message });
        }
        res.json({ data: rows });
    });
});

// START SERVER
app.listen(PORT, () => {
    console.log(`Server running on http://localhost:${PORT}`);
} );
