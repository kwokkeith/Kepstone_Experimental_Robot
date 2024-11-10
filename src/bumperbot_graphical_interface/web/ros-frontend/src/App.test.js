// src/App.test.js
import { render, screen } from '@testing-library/react';
import App from './App';

test('renders login page on initial load', () => {
  render(<App />);
  const loginHeader = screen.getByText(/login/i); // Adjust to match actual text
  expect(loginHeader).toBeInTheDocument();
});
