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
  const [jsonData, setJsonData] = useState(null);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState(false);


  // ---------------------------------------
  // -------------- FUNCTIONS --------------
  // ---------------------------------------
  const handleSpeed = (event) => {
    setInputSpeed(event.target.value);
  };

  const handleSubmit = (event) => {
    event.preventDefault();
    setSpeed(inputSpeed);
    console.log(speed);
  };

  

  /******* SERVER COMMUNICATION *******/
  useEffect(() => {
    fetch('http://localhost:3000')
      .then(response => {
        return response.json();
      })
      .then(jsonData => {
        setJsonData(jsonData);
        setLoading(false);
        console.log(jsonData);
      })
      .catch(err => {
        Error("Error in the communication");
        setLoading(false);
      })
  });

  // ---------------------------------
  // -------------- APP --------------
  // ---------------------------------
  return (
    <div className="App">
      <div className="controlPanel">
        <h1>Drone Control panel</h1>

        <form onSubmit={handleSubmit}>
          <label>Insert input commands</label>
          <input
            type="number"
            onChange={handleSpeed}
            value={inputSpeed}></input>
          <button type="submit">Send</button>
        </form>

        

      </div>
    </div>
  );
}

export default App;
