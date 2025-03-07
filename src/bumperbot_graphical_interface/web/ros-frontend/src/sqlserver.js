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

// Insert data into ContainsTable
app.post('/api/contains', (req, res) => {
    const { job_id, sequence_no } = req.body;
    const query = `INSERT INTO ContainsTable (job_id, sequence_no) VALUES (?, ?)`;
    db.run(query, [job_id, sequence_no], function(err) {
        if (err) {
            console.error('Error inserting into ContainsTable:', err.message);
            return res.status(500).json({ error: err.message });
        }
        res.json({ message: 'ContainsTable entry inserted successfully' });
    });
});

// GET SCHEDULES WITH OPTIONAL DATE FILTERING
app.get('/api/schedules', (req, res) => {
    const { date } = req.query;
    let query = `SELECT * FROM Schedule_Table`;
    let params = [];

    if (date) {
        query += ` WHERE start_date = ?`;
        params.push(date);
    }

    db.all(query, params, (err, rows) => {
        if (err) {
            console.error('Error fetching schedules:', err.message);
            return res.status(500).json({ error: err.message });
        }
        res.json({ data: rows });
    });
});

// app.get('/api/schedule_job_link_withName', (req, res) => {
//     const scheduleName = req.query.schedule_name; // Get the schedule name from query parameters
//     console.log('Schedule Name:', scheduleName); // Debugging statement

//     const query = `
//     WITH DistinctJobs AS (
//         SELECT 
//             schedule_id,
//             MIN(job_id) AS job_id
//         FROM 
//             Schedule_Job_Link
//         GROUP BY 
//             schedule_id
//     )
//     SELECT 
//         DistinctJobs.job_id,
//         Schedule_Table.schedule_id,
//         Schedule_Table.schedule_name,
//         Schedule_Table.start_time,
//         Schedule_Table.start_date,
//         Schedule_Table.recurrence,
//         Config_Table.map_name
//     FROM 
//         Schedule_Table
//     JOIN 
//         DistinctJobs ON Schedule_Table.schedule_id = DistinctJobs.schedule_id
//     JOIN 
//         Jobs_Table ON DistinctJobs.job_id = Jobs_Table.job_id
//     JOIN 
//         Config_Table ON Jobs_Table.config_id = Config_Table.config_id
//     WHERE 
//         Schedule_Table.schedule_name = ?
//     `;

//     db.all(query, [scheduleName], (err, rows) => {
//         if (err) {
//             console.error('Error fetching schedules:', err.message);
//             return res.status(500).json({ error: err.message });
//         }
//         res.json({ data: rows });
//     });
// });

// Delete entries related to a schedule and job
app.delete('/api/delete_schedule', (req, res) => {
    const { schedule_name } = req.body;
    console.log('Received request to delete schedule:', schedule_name);

    db.serialize(() => {
        db.run('BEGIN TRANSACTION', (err) => {
            if (err) {
                console.error('Error starting transaction:', err.message);
                return res.status(500).json({ error: 'Error starting transaction' });
            }

            // Debugging: Log schedule_id values
            const logScheduleIds = `SELECT schedule_id FROM Schedule_Table WHERE schedule_name = ?`;
            db.all(logScheduleIds, [schedule_name], (err, rows) => {
                if (err) {
                    console.error('Error fetching schedule_ids:', err.message);
                } else {
                    console.log('Schedule IDs:', rows);
                }
            });

            // Debugging: Log job_id values
            const logJobIds = `SELECT job_id FROM Schedule_Job_Link WHERE schedule_id IN (SELECT schedule_id FROM Schedule_Table WHERE schedule_name = ?)`;
            db.all(logJobIds, [schedule_name], (err, rows) => {
                if (err) {
                    console.error('Error fetching job_ids:', err.message);
                } else {
                    console.log('Job IDs:', rows);
                }
            });

            const deleteJobStatus = `DELETE FROM JobStatus WHERE job_id IN (SELECT job_id FROM Schedule_Job_Link WHERE schedule_id IN (SELECT schedule_id FROM Schedule_Table WHERE schedule_name = ?))`;
            db.run(deleteJobStatus, [schedule_name], function(err) {
                if (err) {
                    console.error('Error deleting from JobStatus:', err.message);
                    return res.status(500).json({ error: err.message });
                } else {
                    console.log(`Deleted ${this.changes} rows from JobStatus`);
                }
            });

            const deleteScheduleJobLink = `DELETE FROM Schedule_Job_Link WHERE schedule_id IN (SELECT schedule_id FROM Schedule_Table WHERE schedule_name = ?)`;
            db.run(deleteScheduleJobLink, [schedule_name], function(err) {
                if (err) {
                    console.error('Error deleting from Schedule_Job_Link:', err.message);
                    return res.status(500).json({ error: err.message });
                } else {
                    console.log(`Deleted ${this.changes} rows from Schedule_Job_Link`);
                }
            });

            const deleteJobOrder = `DELETE FROM Job_Order_Table WHERE job_id IN (SELECT job_id FROM Schedule_Job_Link WHERE schedule_id IN (SELECT schedule_id FROM Schedule_Table WHERE schedule_name = ?))`;
            db.run(deleteJobOrder, [schedule_name], function(err) {
                if (err) {
                    console.error('Error deleting from Job_Order_Table:', err.message);
                    return res.status(500).json({ error: err.message });
                } else {
                    console.log(`Deleted ${this.changes} rows from Job_Order_Table`);
                }
            });

            const deleteJobs = `DELETE FROM Jobs_Table WHERE job_id IN (SELECT job_id FROM Schedule_Job_Link WHERE schedule_id IN (SELECT schedule_id FROM Schedule_Table WHERE schedule_name = ?))`;
            db.run(deleteJobs, [schedule_name], function(err) {
                if (err) {
                    console.error('Error deleting from Jobs_Table:', err.message);
                    return res.status(500).json({ error: err.message });
                } else {
                    console.log(`Deleted ${this.changes} rows from Jobs_Table`);
                }
            });

            const deleteSchedule = `DELETE FROM Schedule_Table WHERE schedule_name = ?`;
            db.run(deleteSchedule, [schedule_name], function(err) {
                if (err) {
                    console.error('Error deleting from Schedule_Table:', err.message);
                    return res.status(500).json({ error: err.message });
                } else {
                    console.log(`Deleted ${this.changes} rows from Schedule_Table`);
                }
            });

            db.run('COMMIT', (err) => {
                if (err) {
                    console.error('Error committing transaction:', err.message);
                    return res.status(500).json({ error: 'Error committing transaction' });
                }
                res.json({ message: 'Entries deleted successfully' });
            });
        });
    });
});

// START SERVER
app.listen(PORT, () => {
    console.log(`Server running on http://localhost:${PORT}`);
} );
