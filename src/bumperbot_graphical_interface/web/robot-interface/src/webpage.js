// Enter fullscreen by default and allow toggling
function toggleFullscreen() {
    const icon = document.getElementById("fullscreen-icon");
    if (!document.fullscreenElement) {
        document.documentElement.requestFullscreen().then(() => {
            icon.classList.replace("fa-maximize", "fa-minimize");
        }).catch((err) => {
            console.warn("Fullscreen mode not enabled:", err);
        });
    } else {
        document.exitFullscreen().then(() => {
            icon.classList.replace("fa-minimize", "fa-maximize");
        }).catch((err) => {
            console.warn("Fullscreen exit not enabled:", err);
        });
    }
}

// Automatically enter fullscreen on page load
window.addEventListener('load', () => {
    toggleFullscreen();
    document.getElementById("menu-button").style.display = "none"; // Hide menu button initially
});

// Check login credentials
function checkLogin() {
    const username = document.getElementById("username").value;
    const password = document.getElementById("password").value;

    if (username === "admin" && password === "password") {
        document.getElementById("login-page").style.display = "none";
        document.getElementById("main-page").style.display = "block";
        document.getElementById("menu-button").style.display = "block"; // Show menu button after login
        return false;
    } else {
        document.getElementById("login-error").style.display = "block";
        return false;
    }
}

// Toggle the sliding menu
function toggleMenu() {
    const menu = document.getElementById("menu");
    const menuButton = document.getElementById("menu-button");
    menu.classList.toggle("active"); // Toggle the 'active' class to show/hide the menu
    menuButton.classList.toggle("menu-open"); // Toggle 'menu-open' class on the menu button
}

// Show different pages based on menu selection
function showPage(page) {
    const pages = document.querySelectorAll('.page');
    pages.forEach(p => p.style.display = 'none');
    document.getElementById(page + '-page').style.display = 'block';
    toggleMenu(); // Hide menu after selecting a page
}

// Placeholder functions for settings page
function localizeRobot() {
    alert("Local Localization Placeholder");
}

function adjustBrightness() {
    alert("Brightness Adjustment Placeholder");
}
