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

// START SERVER
app.listen(PORT, () => {
    console.log(`Server running on http://localhost:${PORT}`);
} );
