// -------------------------------------
// -------------- MODULES --------------
// -------------------------------------
const express = require('express');

// -----------------------------------------------
// -------------- INITIALIZE SERVER --------------
// -----------------------------------------------
const app = express();
const port = 3000;

// -----------------------------------------
// -------------- MIDDLEWARES --------------
// -----------------------------------------

// Handles JSON data
app.use(express.json());

// Handles connection to device
app.use((req, res, next) => {

  var flag = req.body;

  if (!flag.connection) {
    console.log("Connection lost!\nConnecting to source...");
  } else {
    next();
  }
});

// ---------------------------------------------------
// -------------- POST REQUEST HANDLING --------------
// ---------------------------------------------------

/*******  Data from sensors *******/
app.post('/data', (req, res) => {

  const jsonData = req.body;
  
  // Print on terminal
  console.log('Dati JSON ricevuti:', jsonData);
  
  // Send response 
  res.send('Dati ricevuti con successo!');
});

// --------------------------------------------------
// -------------- GET REQUEST HANDLING --------------
// --------------------------------------------------
app.get('/', (req, res) => {
  res.send("Ciao a tutti");
})

// -------------------------------------------
// -------------- LAUNCH SERVER --------------
// -------------------------------------------
app.listen(port, () => {
  console.log(`Server avviato su http://localhost:${port}`);
});