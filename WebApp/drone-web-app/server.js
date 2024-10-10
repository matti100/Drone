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

var accel;
var gyro;
var motorSpeed;
var sendToEsp32 = false;

// -----------------------------------------
// -------------- MIDDLEWARES --------------
// -----------------------------------------

// JSON file handler
app.use(express.json());
// cors permits client-server communication through different ports
app.use(cors());
// Send data to esp32
app.use((req, res, next) => {

    if (sendToEsp32 == true) {

        fetch("http://10.0.0.18:3002/data", {
            method: "POST",
            headers: {
                "Content-Type": "application/JSON"
            },
            body: JSON.stringify({ "speed": speed }),
        })
            .then(response => response.json())
            .then(data => {
                console.log(data);
            })
            .catch(error => {
                console.error(error);
            })

        sendToEsp32 = false;
    }

    next();
});

// --------------------------------------
// -------------- REQUESTS --------------
// --------------------------------------

/* POST */
// Update Speed
app.post('/updateSpeed', (req, res) => {

    speed = req.body.speed;
    console.log("Speed received: " + speed);

    res.send(JSON.stringify({ "message": "Speed updated successfully" }));

    sendToEsp32 = true;
    console.log(sendToEsp32);
});


// Manage connnection status
app.post('/stopConnection', (req, res) => {

    connectionFlag = req.body.flag;
    if (connectionFlag) {
        console.log("Connection established");
    } else {
        console.log("Connection stopped");
    }

    res.send(JSON.stringify({ "message": "Connection stopped successfully" }));
})


// Receive data from ESP32
app.post('/data', (req, res) => {

    accel = req.body.Accel;
    gyro = req.body.Gyro;
    motorSpeed = req.body.motorSpeed;

    console.log("Acceleration\t" + accel);
    console.log("Gyroscope\t" + gyro);
    console.log("Motor Speed\t" + motorSpeed);

    res.send("Data received successfully");

});

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

    res.send(JSON.stringify({ "connection": connectionFlag }));
})


app.listen(port, () => {
    console.log("Server listening on http://localhost:" + port);
});





