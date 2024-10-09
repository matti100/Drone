// ------------------------------------
// -------------- IMPORT --------------
// ------------------------------------
import './App.css';
import React, { useEffect, useState } from 'react';

// -------------- WEB-APP --------------
function App() {

  // ---------------------------------------
  // -------------- VARIABLES --------------
  // ---------------------------------------

  /******* STATES *******/
  // Speed
  const [speed, setSpeed] = useState(0);
  const [inputSpeed, setInputSpeed] = useState("");
  const [message, setMessage] = useState("Connecting...");
  const [connection, setConnection] = useState(false);
  const [connectionButton, setConnectionButton] = useState("Stop Connection");


  // ---------------------------------------
  // -------------- FUNCTIONS --------------
  // ---------------------------------------

  // Update motor speed
  useEffect(() => {
    if (speed != "") {

      fetch("http://localhost:3001/updateSpeed", {
        method: "POST",
        headers: {
          "Content-Type": "application/JSON"
        },
        body: JSON.stringify({ "speed": speed }),
      })
        .then(response => response.json())
        .then(data => {
          console.log(data.message)
        })
        .catch(error => {
          console.error("Error detected: " + error)
        })

    }
  }, [speed]);

  // Read connection status
  useEffect(() => {
    fetch('http://localhost:3001/', {
      method: "GET",
      headers: {
        "Content-Type": "application/JSON"
      }
    })
      .then(response => response.json())
      .then(data => {
        if (data.connection) {
          //setConnection(true);
          setMessage('Connected to ESP32');
        } else {
          //setConnection(false);
          setMessage('Connecting...');
        }
      })
      .catch(error => {
        console.error(error);
      })
  });

  // Manage connection status
  useEffect(() => {
    fetch('http://localhost:3001/stopConnection', {
      method: "POST",
      headers: {
        "Content-Type": "application/JSON"
      },
      body: JSON.stringify({ "flag": connection })
    })
      .then(response => response.json())
      .then(data => {
        console.log(data.message)

        if (connection) {
          setConnectionButton("Stop Connection");
        } else{
          setConnectionButton("Connect");
        }

      })
      .catch(error => {
        console.error(error)
      })
  }, [connection])




  /******* SERVER COMMUNICATION *******/


  // ---------------------------------
  // -------------- APP --------------
  // ---------------------------------
  return (
    <div className="App">
      <div className="controlPanel">

        <h1>Drone Control panel</h1>


        {<p>{message}</p>}

        <form onSubmit={(e) => {
          e.preventDefault();
          setSpeed(inputSpeed)
        }
        }>
          <label>Insert input commands</label>
          <input
            type="number"
            onChange={(e) => { setInputSpeed(e.target.value) }}
            value={inputSpeed}></input>
          <button type="submit">Send</button>
        </form>

        <button onClick={() => { setConnection(!connection) }}>
          {connectionButton}
        </button>

        <div className='plotBox'>

        </div>



      </div>
    </div>
  );
}

export default App;
