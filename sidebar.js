(function () {
    const scriptUrl = new URL(document.currentScript.src);
    const siteRoot = scriptUrl.href.slice(0, scriptUrl.href.lastIndexOf("/") + 1);

    const sections = [
        {
            title: "Basics",
            links: [
                ["Home", "index.html"],
                ["Course Map", "CourseMap.html"],
                ["Java Start", "Java/JavaStart.html"],
                ["Java Exercise", "Java/JavaExracise.html"],
                ["Git Start", "Git/GitBasic.html"],
                ["Git Team", "Git/GitTeam.html"],
            ],
        },
        {
            title: "WPILib",
            links: [
                ["Introduction", "WPILib/WPILib_Introduction.html"],
                ["Docs & Tutorials", "WPILib/WPILib_Resources.html"],
                ["Project Structure", "WPILib/WPILib_ProjectStructure.html"],
                ["Subsystems", "WPILib/WPILib_Subsystem.html"],
                ["Commands", "WPILib/WPILib_Command.html"],
                ["Scheduler & RobotContainer", "WPILib/WPILib_CommandSchedulerAndRobotContainer.html"],
                ["Button Bindings", "WPILib/WPILib_ButtonBindings.html"],
                ["Units", "WPILib/WPILib_Units.html"],
            ],
        },
        {
            title: "CTRE",
            links: [
                ["TalonFX", "Ctre/CTRE_TalonFX.html"],
                ["PID", "Ctre/CTRE_PID.html"],
                ["Feedforward", "Ctre/CTRE_Feedforward.html"],
                ["PID & Feedforward", "Ctre/CTRE_PID_Feedforward.html"],
            ],
        },
        {
            title: "Robot Code",
            links: [
                ["Sensors", "Sensors/Sensors_Overview.html"],
                ["Mechanisms", "Mechanisms/Mechanisms_Overview.html"],
                ["Autonomous", "Auto/Autonomous_Overview.html"],
            ],
        },
        {
            title: "Advanced",
            links: [
                ["Simulation", "Simulation/Simulation_Overview.html"],
                ["AdvantageKit", "AdvantageKit/AdvantageKit_Overview.html"],
                ["Team Library", "TeamLibrary/TeamLibrary_Overview.html"],
                ["Catalyst Generator", "frc-catalyst-subsystem-generator.html"],
            ],
        },
    ];

    const style = document.createElement("style");
    style.textContent = `
        :root {
            --yteam-sidebar-width: 270px;
        }
        body.yteam-sidebar-ready {
            padding-left: calc(var(--yteam-sidebar-width) + 28px) !important;
        }
        .yteam-sidebar {
            position: fixed;
            inset: 0 auto 0 0;
            width: var(--yteam-sidebar-width);
            background: #101010;
            color: #ffffff;
            border-right: 1px solid #333;
            box-sizing: border-box;
            overflow-y: auto;
            padding: 22px 18px;
            text-align: left;
            z-index: 1000;
        }
        .yteam-sidebar__brand {
            display: flex;
            align-items: center;
            gap: 12px;
            color: #ffffff;
            text-decoration: none;
            font-weight: 700;
            margin-bottom: 24px;
        }
        .yteam-sidebar__brand img {
            width: 44px;
            height: 44px;
            object-fit: cover;
            border-radius: 8px;
            margin: 0;
            box-shadow: none;
        }
        .yteam-sidebar__section {
            margin: 0 0 22px 0;
        }
        .yteam-sidebar__title {
            color: #a8a8a8;
            font-size: 0.78rem;
            font-weight: 700;
            letter-spacing: 0;
            margin: 0 0 8px 0;
            text-transform: uppercase;
        }
        .yteam-sidebar__link {
            display: block;
            color: #d7e9ff;
            text-decoration: none;
            font-weight: 600;
            padding: 8px 10px;
            border-radius: 6px;
            line-height: 1.25;
        }
        .yteam-sidebar__link:hover,
        .yteam-sidebar__link:focus {
            background: #1f3550;
            color: #ffffff;
            text-decoration: none;
            outline: none;
        }
        .yteam-sidebar__link.is-active {
            background: #4da6ff;
            color: #000000;
        }
        .yteam-sidebar-toggle {
            display: none;
            position: fixed;
            top: 12px;
            left: 12px;
            z-index: 1001;
            border: 1px solid #444;
            background: #101010;
            color: #ffffff;
            border-radius: 6px;
            padding: 8px 10px;
            font: 700 16px/1 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            cursor: pointer;
        }
        @media (max-width: 760px) {
            body.yteam-sidebar-ready {
                padding-left: 20px !important;
                padding-top: 64px !important;
            }
            .yteam-sidebar {
                transform: translateX(-100%);
                transition: transform 0.2s ease;
            }
            body.yteam-sidebar-open .yteam-sidebar {
                transform: translateX(0);
            }
            .yteam-sidebar-toggle {
                display: block;
            }
        }
    `;
    document.head.appendChild(style);

    const makeUrl = (path) => new URL(path, siteRoot).href;
    const currentPath = new URL(window.location.href).pathname.replace(/\/+$/, "");

    const sidebar = document.createElement("aside");
    sidebar.className = "yteam-sidebar";
    sidebar.setAttribute("aria-label", "Training navigation");

    const brand = document.createElement("a");
    brand.className = "yteam-sidebar__brand";
    brand.href = makeUrl("index.html");
    brand.innerHTML = `<img src="${makeUrl("pics/YTeamLogo.jpeg")}" alt=""> <span>Y-Team Training</span>`;
    sidebar.appendChild(brand);

    sections.forEach((section) => {
        const group = document.createElement("section");
        group.className = "yteam-sidebar__section";

        const title = document.createElement("h2");
        title.className = "yteam-sidebar__title";
        title.textContent = section.title;
        group.appendChild(title);

        section.links.forEach(([label, path]) => {
            const link = document.createElement("a");
            const href = makeUrl(path);
            const linkPath = new URL(href).pathname.replace(/\/+$/, "");
            link.className = "yteam-sidebar__link";
            link.href = href;
            link.textContent = label;
            if (linkPath === currentPath) {
                link.classList.add("is-active");
                link.setAttribute("aria-current", "page");
            }
            group.appendChild(link);
        });

        sidebar.appendChild(group);
    });

    const toggle = document.createElement("button");
    toggle.className = "yteam-sidebar-toggle";
    toggle.type = "button";
    toggle.setAttribute("aria-label", "Toggle navigation");
    toggle.setAttribute("aria-expanded", "false");
    toggle.textContent = "Menu";
    toggle.addEventListener("click", () => {
        const isOpen = document.body.classList.toggle("yteam-sidebar-open");
        toggle.setAttribute("aria-expanded", String(isOpen));
    });

    document.body.insertBefore(sidebar, document.body.firstChild);
    document.body.insertBefore(toggle, sidebar);
    document.body.classList.add("yteam-sidebar-ready");
})();
