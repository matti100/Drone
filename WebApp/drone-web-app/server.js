// -------------- MODULES --------------
const express = require('express');

// -------------- INITIALIZE SERVER --------------
const app = express();
app.use(express.json());

// Server port
const port = 3000;

// -------------- POST REQUEST HANDLING --------------
app.post('/data', (req, res) => {

  const jsonData = req.body;
  
  // Print on terminal
  console.log('Dati JSON ricevuti:', jsonData);
  
  // Send response 
  res.send('Dati ricevuti con successo!');
});

// -------------- LAUNCH SERVER --------------
app.listen(port, () => {
  console.log(`Server avviato su http://localhost:${port}`);
});