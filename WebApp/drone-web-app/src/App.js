// -------------- IMPORT --------------
import './App.css';
import React, { useEffect, useState } from 'react';

// -------------- VARIABLES --------------
const [data, setData] = useState(null);
const [error, setError] = useState(null);

// -------------- FUNCTIONS --------------
  // Funzione per ottenere i dati dal server
  const fetchData = async () => {
    try {
      const response = await fetch('http://localhost:3000/data');  // URL del server Node.js
      if (!response.ok) {
        throw new Error('Errore nella richiesta');
      }
      const jsonData = await response.json();
      setData(jsonData);  // Imposta i dati ricevuti
    } catch (error) {
      setError(error.message);
    }
  };

  // Effettua la richiesta quando il componente viene caricato
  useEffect(() => {
    fetchData();
  }, []);


// -------------- WEB-APP --------------
function App() {
  return (
    <div className="App">
      <div className="controPanel">
        <h1>Drone Control panel</h1>
        <form>
          <label>Insert input commands</label>
          <input type="number"></input>
        </form>
      </div>
    </div>
  );
}

export default App;
