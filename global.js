// global.js
document.addEventListener("DOMContentLoaded", function() {
    // 1. Create the sidebar HTML structure
    const sidebarHTML = `
    <div class="site-sidebar">
        <h2>The Y Team</h2>
        <a href="index.html">🏠 Home</a>
        <a href="mechanics.html">⚙️ Mechanics & CAD</a>
        <a href="software.html">💻 Control Software</a>
        <a href="outreach.html">🤝 Community Outreach</a>
    </div>
    <style>
        .site-sidebar {
            height: 100vh; width: 250px; position: fixed;
            top: 0; left: 0; background-color: #1e293b;
            padding-top: 30px; color: white; font-family: sans-serif;
            box-shadow: 2px 0 10px rgba(0,0,0,0.2); z-index: 1000;
        }
        .site-sidebar h2 { text-align: center; margin-bottom: 30px; color: #f8fafc; }
        .site-sidebar a { 
            padding: 15px 30px; text-decoration: none; color: #cbd5e1; 
            display: block; transition: 0.2s; border-left: 4px solid transparent; 
        }
        .site-sidebar a:hover { background-color: #334155; color: #38bdf8; border-left: 4px solid #38bdf8; }
        body { margin-left: 250px; } /* Automatically pushes content for every page */
    </style>
    `;

    // 2. Inject it into the very beginning of the <body>
    document.body.insertAdjacentHTML('afterbegin', sidebarHTML);
});