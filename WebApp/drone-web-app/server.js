// -------------- MODULES --------------
const express = require('express');
const app = express();

// -------------- INITIALIZE SERVER --------------
// Middleware per analizzare il corpo delle richieste come JSON
app.use(express.json());

// Server port
const port = 3000;

// POST request handling
app.post('/data', (req, res) => {

  const jsonData = req.body;
  
  // Print on terminal
  console.log('Dati JSON ricevuti:', jsonData);
  
  // Send response 
  res.send('Dati ricevuti con successo!');
});

// Lauch server
app.listen(port, () => {
  console.log(`Server avviato su http://localhost:${port}`);
});