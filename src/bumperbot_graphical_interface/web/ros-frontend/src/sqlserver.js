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
app.get('/api/data', (req, res) => {
    const query = `SELECT * FROM messages`;
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
    const sqlDelete = `DELETE FROM messages WHERE map_name = ?`;

    db.run(sqlDelete, [mapName], function(err) {
        if (err) {
            console.error('Error running SQL:', err.message);
            res.status(500).json({ error: 'Database error' });
            return;
        }
        if (this.changes === 0) {
            res.status(404).json({ message: 'No record found to delete.' });
            return;
        }
        res.json({ message: `Row(s) deleted: ${this.changes}` });
    });
});

// START SERVER
app.listen(PORT, () => {
    console.log(`Server running on http://localhost:${PORT}`);
} );
