// -------------------------------------
// -------------- MODULES --------------
// -------------------------------------
const express = require("express");
const cors = require("cors");

// -----------------------------------------------
// -------------- INITIALIZE SERVER --------------
// -----------------------------------------------
const app = express();
const port = 3001;

// ---------------------------------------
// -------------- VARIABLES --------------
// ---------------------------------------
var connectionFlag;
var speed;


// -----------------------------------------
// -------------- MIDDLEWARES --------------
// -----------------------------------------

// JSON file handler
app.use(express.json());
// cors permits client-server communication through different ports
app.use(cors());

// --------------------------------------
// -------------- REQUESTS --------------
// --------------------------------------

/* POST */
// Update Speed
app.post('/updateSpeed', (req, res) => {

    speed = req.body.speed;
    console.log("Speed received: " + speed);

    res.send(JSON.stringify({"message": "Speed updated successfully"}));
});


// Manage connnection status
app.post('/stopConnection', (req, res) => {

    connectionFlag = req.body.flag;
    if (connectionFlag){
        console.log("Connection established");
    } else {
        console.log("Connection stopped");
    }

    res.send(JSON.stringify({"message": "Connection stopped successfully"}));
})

/* GET */
// Manage connection status
app.get('/', (req, res) => {
    /* 
    if (connectionFlag == false) {
        console.log("Connecting...");
    } else {
        console.log("Connection established !"); 
    }
    */ 

    res.send(JSON.stringify({"connection": connectionFlag}));
})



app.listen(port, () => {
    console.log("Server listening on http://localhost:" + port);
});





