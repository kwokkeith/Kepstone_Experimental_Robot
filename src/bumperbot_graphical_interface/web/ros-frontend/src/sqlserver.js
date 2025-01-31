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
    
    // Start a transaction 
    db.serialize(()=>{
        db.run('BEGIN TRANSACTION');

        // Delete from Polygon table based on config_id
        const sqlDeletePolygon = `DELETE FROM Polygon_Table
        WHERE config_id IN (SELECT config_id FROM Config_Table WHERE map_name = ?)`;

        db.run(sqlDeletePolygon, [mapName], function(err) {
            if (err) {
                db.run('ROLLBACK');
                // console.error('Error running SQL:', err.message);
                res.status(500).json({ error: 'Database error' });
                return;
            }

            const sqlDeleteConfig = `DELETE FROM Config_Table WHERE map_name = ?`;

            db.run(sqlDeleteConfig, [mapName], function(err) {
                if (err) {
                    db.run('ROLLBACK');
                    res.status(500).json({ error: 'Database error' });
                    return;
                }
                db.run('COMMIT', (err) => {
                    if (err) {
                        res.status(500).json({ error: 'Database error' });
                        return;
                    }
                    if (this.changes === 0) {
                        res.status(404).json({ message: 'No record found to delete.' });
                        return;
                    }
                    res.json({ message: `Map with name ${mapName} deleted`, changes: this.changes });
                });
            });
        });
    });
});

// START SERVER
app.listen(PORT, () => {
    console.log(`Server running on http://localhost:${PORT}`);
} );
