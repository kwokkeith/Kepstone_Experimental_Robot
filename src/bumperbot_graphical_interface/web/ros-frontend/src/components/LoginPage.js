// src/components/LoginPage.js
import React, { useState } from 'react';
import './LoginPage.css';

const LoginPage = ({ onLoginSuccess }) => {
  const [username, setUsername] = useState('admin');
  const [password, setPassword] = useState('password');
  const [error, setError] = useState(false);

  const handleLogin = (event) => {
    event.preventDefault();
    if (!username || !password) {
      setError('Please fill in all fields.');
    } else if (username !== 'admin' || password !== 'password') {
      setError('Incorrect username or password.');
    } else {
      onLoginSuccess();
    }
  };

  return (
    <div id="login-page" className="page">
      <h2>Login</h2>
      <form onSubmit={handleLogin}>
        <div className="input-group">
          <label htmlFor="username">Username</label>
          <input
            type="text"
            id="username"
            aria-label="Username"
            placeholder="Enter your username"
            value={username}
            onChange={(e) => setUsername(e.target.value)}
            required
          />
        </div>
        
        <div className="input-group">
          <label htmlFor="password">Password</label>
          <input
            type="password"
            id="password"
            placeholder="Enter your password"
            value={password}
            onChange={(e) => setPassword(e.target.value)}
            required
          />
        </div>

        {/* Move the button-container back inside the form */}
        <div className="button-container">
          <button type="submit">Login</button>
        </div>
      </form>

      {error && <p id="login-error">Incorrect login, please try again.</p>}
    </div>
  );
};

export default LoginPage;
