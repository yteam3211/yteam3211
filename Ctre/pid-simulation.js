(function () {
    // Simulation math is adapted from WPILib's BSD-licensed Java classes:
    // DCMotor, LinearSystemId, LinearSystemSim, FlywheelSim, SingleJointedArmSim,
    // PIDController, SimpleMotorFeedforward, and ArmFeedforward.
    const kDtSeconds = 0.02;
    const kBatteryVoltage = 12;
    const kGravity = 9.8;
    const kTwoPi = Math.PI * 2;

    function clamp(value, min, max) {
        return Math.max(min, Math.min(max, value));
    }

    function signum(value) {
        if (value > 0) {
            return 1;
        }
        if (value < 0) {
            return -1;
        }
        return 0;
    }

    function rpmToRadPerSec(rpm) {
        return rpm * kTwoPi / 60;
    }

    function radPerSecToRps(radPerSec) {
        return radPerSec / kTwoPi;
    }

    function rpsToRadPerSec(rps) {
        return rps * kTwoPi;
    }

    function degToRad(degrees) {
        return degrees * Math.PI / 180;
    }

    function radToDeg(radians) {
        return radians * 180 / Math.PI;
    }

    function format(value) {
        if (Math.abs(value) < 0.1) {
            return value.toFixed(3);
        }
        if (Math.abs(value) >= 100) {
            return value.toFixed(0);
        }
        if (Math.abs(value) >= 10) {
            return value.toFixed(1);
        }
        return value.toFixed(2);
    }

    function gaussianNoise(stdDev) {
        if (stdDev <= 0) {
            return 0;
        }
        const u1 = Math.max(Math.random(), Number.EPSILON);
        const u2 = Math.random();
        return Math.sqrt(-2 * Math.log(u1)) * Math.cos(kTwoPi * u2) * stdDev;
    }

    function rk4(state, input, dt, derivative) {
        const addScaled = (base, slope, scale) => base.map((value, index) => value + slope[index] * scale);
        const k1 = derivative(state, input);
        const k2 = derivative(addScaled(state, k1, dt / 2), input);
        const k3 = derivative(addScaled(state, k2, dt / 2), input);
        const k4 = derivative(addScaled(state, k3, dt), input);
        return state.map((value, index) => value + dt / 6 * (k1[index] + 2 * k2[index] + 2 * k3[index] + k4[index]));
    }

    // Small browser port of the WPILib math classes used by FlywheelSim and SingleJointedArmSim.
    class DCMotor {
        constructor(nominalVoltageVolts, stallTorqueNewtonMeters, stallCurrentAmps, freeCurrentAmps, freeSpeedRadPerSec, numMotors) {
            this.nominalVoltageVolts = nominalVoltageVolts;
            this.stallTorqueNewtonMeters = stallTorqueNewtonMeters * numMotors;
            this.stallCurrentAmps = stallCurrentAmps * numMotors;
            this.freeCurrentAmps = freeCurrentAmps * numMotors;
            this.freeSpeedRadPerSec = freeSpeedRadPerSec;
            this.rOhms = nominalVoltageVolts / this.stallCurrentAmps;
            this.KvRadPerSecPerVolt = freeSpeedRadPerSec / (nominalVoltageVolts - this.rOhms * this.freeCurrentAmps);
            this.KtNMPerAmp = this.stallTorqueNewtonMeters / this.stallCurrentAmps;
        }

        static getKrakenX60(numMotors) {
            return new DCMotor(12, 7.09, 366, 2, rpmToRadPerSec(6000), numMotors);
        }

        getCurrent(speedRadPerSec, voltageInputVolts) {
            return -speedRadPerSec / this.KvRadPerSecPerVolt / this.rOhms + voltageInputVolts / this.rOhms;
        }
    }

    class LinearSystem {
        constructor(a, b) {
            this.a = a;
            this.b = b;
        }

        derivative(x, u) {
            return this.a.map((row, rowIndex) => row.reduce((sum, coefficient, columnIndex) => sum + coefficient * x[columnIndex], 0) + this.b[rowIndex] * u);
        }
    }

    const LinearSystemId = {
        createFlywheelSystem(motor, jKgMetersSquared, gearing) {
            const a = -gearing * gearing * motor.KtNMPerAmp / (motor.KvRadPerSecPerVolt * motor.rOhms * jKgMetersSquared);
            const b = gearing * motor.KtNMPerAmp / (motor.rOhms * jKgMetersSquared);
            return new LinearSystem([[a]], [b]);
        },

        createSingleJointedArmSystem(motor, jKgMetersSquared, gearing) {
            const a22 = -gearing * gearing * motor.KtNMPerAmp / (motor.KvRadPerSecPerVolt * motor.rOhms * jKgMetersSquared);
            const b2 = gearing * motor.KtNMPerAmp / (motor.rOhms * jKgMetersSquared);
            return new LinearSystem([[0, 1], [0, a22]], [0, b2]);
        },
    };

    class LinearSystemSim {
        constructor(plant, measurementStdDevs) {
            this.plant = plant;
            this.measurementStdDevs = measurementStdDevs || [];
            this.x = plant.a.map(() => 0);
            this.u = 0;
            this.y = this.x.slice();
        }

        setInput(volts) {
            this.u = volts;
        }

        clampInput(maxInput) {
            this.u = clamp(this.u, -maxInput, maxInput);
        }

        setState(...state) {
            this.x = state.slice();
            this.calculateY();
        }

        update(dtSeconds) {
            this.x = this.updateX(this.x, this.u, dtSeconds);
            this.calculateY();
        }

        updateX(currentState, input, dtSeconds) {
            return rk4(currentState, input, dtSeconds, (x, u) => this.plant.derivative(x, u));
        }

        calculateY() {
            this.y = this.x.map((value, index) => value + gaussianNoise(this.measurementStdDevs[index] || 0));
        }

        getOutput(index) {
            return this.y[index];
        }
    }

    class FlywheelSim extends LinearSystemSim {
        constructor(gearbox, jKgMetersSquared, gearing, measurementStdDevRadPerSec) {
            super(LinearSystemId.createFlywheelSystem(gearbox, jKgMetersSquared, gearing), [measurementStdDevRadPerSec || 0]);
            this.gearbox = gearbox;
            this.gearing = gearing;
            this.jKgMetersSquared = jKgMetersSquared;
        }

        setInputVoltage(volts) {
            this.setInput(volts);
            this.clampInput(kBatteryVoltage);
        }

        setAngularVelocity(radPerSec) {
            this.setState(radPerSec);
        }

        getAngularVelocityRadPerSec() {
            return this.getOutput(0);
        }

        getAngularAccelerationRadPerSecSq() {
            return this.plant.derivative(this.x, this.u)[0];
        }

        getCurrentDrawAmps() {
            return this.gearbox.getCurrent(this.x[0] * this.gearing, this.u) * signum(this.u);
        }
    }

    class SingleJointedArmSim extends LinearSystemSim {
        constructor(gearbox, gearing, jKgMetersSquared, armLengthMeters, minAngleRads, maxAngleRads, simulateGravity, startingAngleRads, measurementStdDevRads) {
            super(LinearSystemId.createSingleJointedArmSystem(gearbox, jKgMetersSquared, gearing), [measurementStdDevRads || 0, 0]);
            this.gearbox = gearbox;
            this.gearing = gearing;
            this.armLengthMeters = armLengthMeters;
            this.minAngleRads = minAngleRads;
            this.maxAngleRads = maxAngleRads;
            this.simulateGravity = simulateGravity;
            this.setState(startingAngleRads, 0);
        }

        static estimateMOI(lengthMeters, massKg) {
            return 1 / 3 * massKg * lengthMeters * lengthMeters;
        }

        setInputVoltage(volts) {
            this.setInput(volts);
            this.clampInput(kBatteryVoltage);
        }

        setState(angleRads, velocityRadPerSec) {
            super.setState(clamp(angleRads, this.minAngleRads, this.maxAngleRads), velocityRadPerSec);
        }

        wouldHitLowerLimit(angleRads) {
            return angleRads <= this.minAngleRads;
        }

        wouldHitUpperLimit(angleRads) {
            return angleRads >= this.maxAngleRads;
        }

        getAngleRads() {
            return this.getOutput(0);
        }

        getVelocityRadPerSec() {
            return this.getOutput(1);
        }

        getCurrentDrawAmps() {
            return this.gearbox.getCurrent(this.x[1] * this.gearing, this.u) * signum(this.u);
        }

        updateX(currentState, input, dtSeconds) {
            const updated = rk4(currentState, input, dtSeconds, (x, u) => {
                const xdot = this.plant.derivative(x, u);
                if (this.simulateGravity) {
                    xdot[1] += 1.5 * -kGravity * Math.cos(x[0]) / this.armLengthMeters;
                }
                return xdot;
            });

            if (this.wouldHitLowerLimit(updated[0])) {
                return [this.minAngleRads, 0];
            }
            if (this.wouldHitUpperLimit(updated[0])) {
                return [this.maxAngleRads, 0];
            }
            return updated;
        }
    }

    class PIDController {
        constructor(kp, ki, kd, periodSeconds) {
            this.period = periodSeconds || kDtSeconds;
            this.minimumIntegral = -1;
            this.maximumIntegral = 1;
            this.iZone = Number.POSITIVE_INFINITY;
            this.setPID(kp, ki, kd);
            this.reset();
        }

        setPID(kp, ki, kd) {
            this.kp = kp;
            this.ki = ki;
            this.kd = kd;
        }

        calculate(measurement, setpoint) {
            this.setpoint = setpoint;
            this.prevError = this.error;
            this.error = this.setpoint - measurement;
            this.errorDerivative = (this.error - this.prevError) / this.period;

            if (Math.abs(this.error) > this.iZone) {
                this.totalError = 0;
            } else if (this.ki !== 0) {
                this.totalError = clamp(
                    this.totalError + this.error * this.period,
                    this.minimumIntegral / this.ki,
                    this.maximumIntegral / this.ki
                );
            }

            return this.kp * this.error + this.ki * this.totalError + this.kd * this.errorDerivative;
        }

        reset() {
            this.error = 0;
            this.prevError = 0;
            this.totalError = 0;
            this.errorDerivative = 0;
        }
    }

    class SimpleMotorFeedforward {
        constructor(ks, kv, ka, dtSeconds) {
            this.ks = ks;
            this.kv = kv;
            this.ka = ka || 0;
            this.dt = dtSeconds || kDtSeconds;
        }

        calculate(velocity, acceleration) {
            return this.ks * signum(velocity) + this.kv * velocity + this.ka * (acceleration || 0);
        }

        calculateWithVelocities(currentVelocity, nextVelocity) {
            if (this.ka < 1e-9) {
                return this.ks * signum(nextVelocity) + this.kv * nextVelocity;
            }
            const a = -this.kv / this.ka;
            const b = 1 / this.ka;
            const ad = Math.exp(a * this.dt);
            const bd = a > -1e-9 ? b * this.dt : 1 / a * (ad - 1) * b;
            return this.ks * signum(currentVelocity) + (nextVelocity - ad * currentVelocity) / bd;
        }
    }

    class ArmFeedforward {
        constructor(ks, kg, kv, ka, dtSeconds) {
            this.ks = ks;
            this.kg = kg;
            this.kv = kv;
            this.ka = ka || 0;
            this.dt = dtSeconds || kDtSeconds;
        }

        calculate(positionRadians, velocityRadPerSec, accelRadPerSecSquared) {
            return this.ks * signum(velocityRadPerSec)
                + this.kg * Math.cos(positionRadians)
                + this.kv * velocityRadPerSec
                + this.ka * (accelRadPerSecSquared || 0);
        }
    }

    const modes = {
        arm: {
            title: "Vertical Arm Position Tuning",
            targetLabel: "Goal angle",
            targetUnit: "deg",
            minTarget: -65,
            maxTarget: 85,
            defaultTarget: 35,
            startPosition: -45,
            hint: "Click or drag around the arm arc to command a new angle. This uses the same gravity term shape as WPILib SingleJointedArmSim, with a lighter gearing setup so aggressive gains overshoot.",
            ranges: {
                kP: [0, 20, 0.1],
                kI: [0, 2, 0.01],
                kD: [0, 5, 0.05],
                kG: [0, 8, 0.05],
                kV: [0, 6, 0.05],
                noise: [0, 1.5, 0.05],
            },
            presets: {
                zero: { kP: 0, kI: 0, kD: 0, kG: 0, kV: 0, noise: 0.05, strategy: "feedback" },
            },
        },
        flywheel: {
            title: "Flywheel Velocity Tuning",
            targetLabel: "Goal speed",
            targetUnit: "rps",
            minTarget: 0,
            maxTarget: 120,
            defaultTarget: 70,
            startPosition: 0,
            hint: "Use the ball disturbance to see why feedforward plus feedback recovers better than either one alone. Too much kV feedforward will drive past the target speed.",
            ranges: {
                kP: [0, 2, 0.01],
                kI: [0, 0.12, 0.001],
                kD: [0, 0.6, 0.005],
                kG: [0, 4, 0.05],
                kV: [0, 0.35, 0.002],
                noise: [0, 1.5, 0.05],
            },
            presets: {
                zero: { kP: 0, kI: 0, kD: 0, kG: 0, kV: 0, noise: 0.05, strategy: "feedforward" },
            },
        },
    };

    const controlIds = ["target", "kP", "kI", "kD", "kG", "kV", "noise"];
    const controlLabels = {
        target: "Setpoint",
        kP: "kP feedback",
        kI: "kI feedback",
        kD: "kD damping",
        kG: "kG / kS feedforward",
        kV: "kV feedforward",
        noise: "Sensor noise",
    };

    const scopeConfig = {
        pid: {
            strategies: ["feedback"],
            hiddenControls: ["kG", "kV"],
            defaultMode: "arm",
            defaultPreset: "zero",
            defaultStrategy: "feedback",
            hint: "PID-only: tune kP first, then add kD to reduce overshoot. Feedforward controls are hidden on this page.",
        },
        feedforward: {
            strategies: ["feedforward"],
            hiddenControls: ["kP", "kI", "kD"],
            defaultMode: "flywheel",
            defaultPreset: "zero",
            defaultStrategy: "feedforward",
            hint: "Feedforward-only: tune the prediction gains. Try a disturbance to see that feedforward does not correct unexpected error by itself.",
        },
        combined: {
            strategies: ["feedback", "feedforward", "combined"],
            hiddenControls: [],
            defaultMode: "flywheel",
            defaultPreset: "zero",
            defaultStrategy: "combined",
            hint: "Combined control: compare feedback, feedforward, and both together. This is what most tuned FRC mechanisms are aiming for.",
        },
    };

    function buildSimulation(root) {
        const scope = root.dataset.scope || (root.dataset.preset === "feedforward" ? "combined" : "pid");
        const config = scopeConfig[scope] || scopeConfig.pid;
        const initialMode = config.defaultMode;
        const sim = {
            mode: initialMode,
            strategy: config.defaultStrategy,
            running: true,
            smooth: false,
            dragging: false,
            goal: modes[initialMode].defaultTarget,
            setpoint: modes[initialMode].defaultTarget,
            previousSetpoint: modes[initialMode].defaultTarget,
            output: modes[initialMode].startPosition,
            velocity: 0,
            voltage: 0,
            currentDraw: 0,
            previousErrorForStatus: 0,
            overshootTimer: 0,
            history: [],
            lastTime: performance.now(),
            plant: null,
            pid: new PIDController(0, 0, 0, kDtSeconds),
        };

        root.innerHTML = `
            <div class="pid-sim__header">
                <div>
                    <h2 data-title></h2>
                    <p data-hint></p>
                </div>
                <div class="pid-sim__actions">
                    <button type="button" data-action="toggle">Pause</button>
                    <button type="button" data-action="reset">Reset</button>
                    <button type="button" data-action="disturb">Disturb</button>
                </div>
            </div>

            <div class="pid-sim__toolbar">
                <div class="pid-sim__switches" aria-label="Simulation mechanism">
                    <button type="button" data-mode="arm">Arm</button>
                    <button type="button" data-mode="flywheel">Flywheel</button>
                </div>
                <div class="pid-sim__switches" aria-label="Control strategy">
                    <button type="button" data-strategy="feedback">Feedback</button>
                    <button type="button" data-strategy="feedforward">Feedforward</button>
                    <button type="button" data-strategy="combined">Combined</button>
                </div>
            </div>

            <div class="pid-sim__layout">
                <div>
                    <div class="pid-sim__canvas-wrap">
                        <canvas aria-label="PID tuning simulation" role="img"></canvas>
                    </div>
                    <p class="pid-sim__lesson" data-lesson></p>
                </div>
                <div class="pid-sim__controls">
                    ${controlIds.map((id) => `
                        <div class="pid-sim__control">
                            <label for="pid-sim-${id}">
                                <span data-label="${id}">${controlLabels[id]}</span>
                                <output data-output="${id}"></output>
                            </label>
                            <input id="pid-sim-${id}" data-control="${id}" type="number" inputmode="decimal" required>
                        </div>
                    `).join("")}
                    <label class="pid-sim__check">
                        <input type="checkbox" data-control="smooth">
                        Smooth setpoint
                    </label>
                </div>
            </div>

            <div class="pid-sim__readouts">
                <div class="pid-sim__readout"><span>Setpoint</span><strong data-readout="setpoint"></strong></div>
                <div class="pid-sim__readout"><span>Output</span><strong data-readout="output"></strong></div>
                <div class="pid-sim__readout"><span>Error</span><strong data-readout="error"></strong></div>
                <div class="pid-sim__readout"><span>Voltage</span><strong data-readout="voltage"></strong></div>
                <div class="pid-sim__readout"><span>Status</span><strong data-readout="status"></strong></div>
            </div>
        `;

        const canvas = root.querySelector("canvas");
        const context = canvas.getContext("2d");
        const inputs = {};

        root.querySelectorAll("[data-control]").forEach((input) => {
            inputs[input.dataset.control] = input;
            input.addEventListener("input", () => {
                if (input.dataset.control === "smooth") {
                    sim.smooth = input.checked;
                    return;
                }
                normalizeNumberInput(input);
                if (input.dataset.control === "target") {
                    sim.goal = Number(input.value);
                    if (!sim.smooth) {
                        sim.setpoint = sim.goal;
                    }
                }
                updateOutputs();
                updatePlantNoise();
            });
        });

        root.querySelectorAll("[data-mode]").forEach((button) => {
            button.addEventListener("click", () => {
                setMode(button.dataset.mode);
            });
        });

        root.querySelectorAll("[data-strategy]").forEach((button) => {
            button.addEventListener("click", () => {
                sim.strategy = button.dataset.strategy;
                updateActiveButtons();
            });
        });

        root.querySelector("[data-action='toggle']").addEventListener("click", (event) => {
            sim.running = !sim.running;
            event.currentTarget.textContent = sim.running ? "Pause" : "Run";
            sim.lastTime = performance.now();
        });

        root.querySelector("[data-action='reset']").addEventListener("click", () => {
            resetState();
        });

        root.querySelector("[data-action='disturb']").addEventListener("click", () => {
            if (sim.mode === "flywheel") {
                sim.plant.setAngularVelocity(sim.plant.x[0] * 0.68);
            } else {
                sim.plant.setState(sim.plant.x[0], sim.plant.x[1] - 3.5);
            }
        });

        canvas.addEventListener("pointerdown", (event) => {
            sim.dragging = true;
            canvas.setPointerCapture(event.pointerId);
            setGoalFromPointer(event);
        });

        canvas.addEventListener("pointermove", (event) => {
            if (sim.dragging) {
                setGoalFromPointer(event);
            }
        });

        canvas.addEventListener("pointerup", (event) => {
            sim.dragging = false;
            canvas.releasePointerCapture(event.pointerId);
        });

        function value(id) {
            return Number(inputs[id].value);
        }

        function setControl(id, valueToSet) {
            inputs[id].value = valueToSet;
        }

        function currentMode() {
            return modes[sim.mode];
        }

        function setMode(mode) {
            sim.mode = mode;
            const modeConfig = currentMode();
            sim.goal = modeConfig.defaultTarget;
            sim.setpoint = modeConfig.defaultTarget;
            sim.strategy = config.defaultStrategy;
            configureControlRanges();
            applyPreset(config.defaultPreset, true);
            resetState();
        }

        function configureControlRanges() {
            const modeConfig = currentMode();
            const target = inputs.target;
            target.min = modeConfig.minTarget;
            target.max = modeConfig.maxTarget;
            target.step = 1;
            target.value = sim.goal;

            Object.keys(modeConfig.ranges).forEach((id) => {
                const [, , step] = modeConfig.ranges[id];
                inputs[id].step = step;
            });

            root.querySelector("[data-label='target']").textContent = modeConfig.targetLabel;
            root.querySelector("[data-title]").textContent = modeConfig.title;
            root.querySelector("[data-hint]").textContent = `${modeConfig.hint} ${config.hint}`;
            root.querySelector("[data-action='disturb']").textContent = sim.mode === "flywheel" ? "Shoot ball" : "Bump arm";
            applyScopeVisibility();
        }

        function applyScopeVisibility() {
            controlIds.forEach((id) => {
                const input = inputs[id];
                const control = input.closest(".pid-sim__control");
                control.style.display = config.hiddenControls.includes(id) ? "none" : "";
            });

            root.querySelectorAll("[data-strategy]").forEach((button) => {
                button.style.display = config.strategies.includes(button.dataset.strategy) ? "" : "none";
            });

            const strategyGroup = root.querySelector("[aria-label='Control strategy']");
            strategyGroup.style.display = config.strategies.length > 1 ? "" : "none";
        }

        function applyPreset(name, keepStrategy) {
            const preset = currentMode().presets[name];
            Object.keys(preset).forEach((id) => {
                if (id === "strategy") {
                    if (!keepStrategy) {
                        sim.strategy = preset.strategy;
                    }
                } else {
                    setControl(id, preset[id]);
                }
            });
            updateOutputs();
            updateActiveButtons();
        }

        function normalizeNumberInput(input) {
            if (input.type !== "number") {
                return;
            }

            const number = Number(input.value);

            if (input.value === "" || !Number.isFinite(number)) {
                input.setCustomValidity("Enter a number.");
                return;
            }

            input.setCustomValidity("");
        }

        function resetState() {
            const modeConfig = currentMode();
            sim.goal = Number(inputs.target.value) || modeConfig.defaultTarget;
            sim.setpoint = sim.goal;
            sim.previousSetpoint = sim.setpoint;
            sim.pid.reset();
            sim.voltage = 0;
            sim.currentDraw = 0;
            sim.previousErrorForStatus = 0;
            sim.overshootTimer = 0;
            sim.history = [];
            sim.lastTime = performance.now();
            createPlant();
            updateOutputs();
        }

        function createPlant() {
            const motor = DCMotor.getKrakenX60(1);
            if (sim.mode === "arm") {
                const armLengthMeters = 0.65;
                const armMassKg = 4.0;
                sim.plant = new SingleJointedArmSim(
                    motor,
                    20,
                    SingleJointedArmSim.estimateMOI(armLengthMeters, armMassKg),
                    armLengthMeters,
                    degToRad(currentMode().minTarget),
                    degToRad(currentMode().maxTarget),
                    true,
                    degToRad(currentMode().startPosition),
                    degToRad(value("noise"))
                );
            } else {
                sim.plant = new FlywheelSim(motor, 0.02, 1, rpsToRadPerSec(value("noise")));
                sim.plant.setAngularVelocity(rpsToRadPerSec(currentMode().startPosition));
            }
        }

        function updatePlantNoise() {
            if (!sim.plant) {
                return;
            }
            sim.plant.measurementStdDevs[0] = sim.mode === "arm" ? degToRad(value("noise")) : rpsToRadPerSec(value("noise"));
        }

        function updateOutputs() {
            const unit = currentMode().targetUnit;
            controlIds.forEach((id) => {
                const output = root.querySelector(`[data-output="${id}"]`);
                const suffix = id === "target" ? ` ${unit}` : "";
                output.textContent = `${format(value(id))}${suffix}`;
            });
        }

        function updateActiveButtons() {
            root.querySelectorAll("[data-mode]").forEach((button) => {
                button.classList.toggle("is-active", button.dataset.mode === sim.mode);
            });
            root.querySelectorAll("[data-strategy]").forEach((button) => {
                button.classList.toggle("is-active", button.dataset.strategy === sim.strategy);
            });
        }

        function setGoalFromPointer(event) {
            const rect = canvas.getBoundingClientRect();
            const x = event.clientX - rect.left;
            const y = event.clientY - rect.top;
            const { width, height } = resizeCanvas();

            if (sim.mode === "arm") {
                const center = armCenter(width, height);
                const angle = Math.atan2(center.y - y, x - center.x) * 180 / Math.PI;
                sim.goal = clamp(angle, currentMode().minTarget, currentMode().maxTarget);
            } else {
                const graph = graphRect(width, height);
                const ratio = clamp(1 - (y - graph.y) / graph.height, 0, 1);
                sim.goal = currentMode().minTarget + ratio * (currentMode().maxTarget - currentMode().minTarget);
            }

            inputs.target.value = Math.round(sim.goal);
            if (!sim.smooth) {
                sim.setpoint = sim.goal;
            }
            updateOutputs();
        }

        function runController(dt) {
            updateSetpoint(dt);
            const measurement = getMeasurementForController();
            const setpoint = getSetpointForController();
            const pidVolts = sim.strategy === "feedforward" ? 0 : pidOutput(measurement, setpoint);
            const ffVolts = sim.strategy === "feedback" ? 0 : feedforwardOutput(dt);
            const voltage = clamp(pidVolts + ffVolts, -kBatteryVoltage, kBatteryVoltage);

            sim.plant.setInputVoltage(voltage);
            sim.plant.update(dt);
            updatePlantOutputs(voltage);
            updateOvershootStatus(dt);
            recordHistory();
        }

        function updateSetpoint(dt) {
            sim.previousSetpoint = sim.setpoint;
            if (!sim.smooth) {
                sim.setpoint = sim.goal;
                return;
            }

            const maxRate = sim.mode === "arm" ? 55 : 45;
            const delta = clamp(sim.goal - sim.setpoint, -maxRate * dt, maxRate * dt);
            sim.setpoint += delta;
        }

        function getMeasurementForController() {
            return sim.mode === "arm" ? sim.plant.getAngleRads() : radPerSecToRps(sim.plant.getAngularVelocityRadPerSec());
        }

        function getSetpointForController() {
            return sim.mode === "arm" ? degToRad(sim.setpoint) : sim.setpoint;
        }

        function pidOutput(measurement, setpoint) {
            sim.pid.setPID(value("kP"), value("kI"), value("kD"));
            return sim.pid.calculate(measurement, setpoint);
        }

        function feedforwardOutput(dt) {
            if (sim.mode === "arm") {
                const currentAngle = degToRad(sim.setpoint);
                const currentVelocity = degToRad((sim.goal - sim.output) * 0.9);
                const armFF = new ArmFeedforward(0, value("kG"), value("kV"), 0, dt);
                return armFF.calculate(currentAngle, currentVelocity, 0);
            }

            const currentVelocity = radPerSecToRps(sim.plant.x[0]);
            const nextVelocity = sim.setpoint;
            const flywheelFF = new SimpleMotorFeedforward(value("kG"), value("kV"), 0, dt);
            return flywheelFF.calculateWithVelocities(currentVelocity, nextVelocity);
        }

        function updatePlantOutputs(voltage) {
            if (sim.mode === "arm") {
                sim.output = radToDeg(sim.plant.x[0]);
                sim.velocity = radToDeg(sim.plant.x[1]);
            } else {
                sim.output = radPerSecToRps(sim.plant.x[0]);
                sim.velocity = radPerSecToRps(sim.plant.getAngularAccelerationRadPerSecSq());
            }
            sim.voltage = voltage;
            sim.currentDraw = sim.plant.getCurrentDrawAmps();
        }

        function updateOvershootStatus(dt) {
            const error = sim.setpoint - sim.output;
            if (
                sim.previousErrorForStatus !== 0
                && Math.sign(error) !== Math.sign(sim.previousErrorForStatus)
                && Math.abs(error) > (sim.mode === "arm" ? 0.4 : 0.8)
            ) {
                sim.overshootTimer = 1.2;
            } else {
                sim.overshootTimer = Math.max(0, sim.overshootTimer - dt);
            }
            sim.previousErrorForStatus = error;
        }

        function recordHistory() {
            sim.history.push({
                setpoint: sim.setpoint,
                output: sim.output,
                voltage: sim.voltage,
            });
            if (sim.history.length > 260) {
                sim.history.shift();
            }
        }

        function resizeCanvas() {
            const ratio = window.devicePixelRatio || 1;
            const width = Math.max(340, Math.floor(canvas.clientWidth));
            const height = Math.max(340, Math.floor(canvas.clientHeight));
            if (canvas.width !== width * ratio || canvas.height !== height * ratio) {
                canvas.width = width * ratio;
                canvas.height = height * ratio;
                context.setTransform(ratio, 0, 0, ratio, 0, 0);
            }
            return { width, height };
        }

        function draw() {
            const { width, height } = resizeCanvas();
            context.clearRect(0, 0, width, height);
            context.fillStyle = "#080808";
            context.fillRect(0, 0, width, height);

            if (sim.mode === "arm") {
                drawArm(width, height);
            } else {
                drawFlywheel(width, height);
            }
            drawGraph(width, height);
            updateReadouts();
        }

        function armCenter(width, height) {
            return {
                x: Math.min(160, width * 0.28),
                y: height * 0.58,
            };
        }

        function graphRect(width, height) {
            const left = Math.min(260, Math.max(210, width * 0.38));
            return {
                x: left,
                y: 36,
                width: width - left - 22,
                height: height - 106,
            };
        }

        function drawArm(width, height) {
            const center = armCenter(width, height);
            const radius = Math.min(118, width * 0.23, height * 0.32);
            const current = degToRad(sim.output);
            const target = degToRad(sim.setpoint);

            context.strokeStyle = "#2f2f2f";
            context.lineWidth = 2;
            context.beginPath();
            context.arc(center.x, center.y, radius, degToRad(-80), degToRad(95));
            context.stroke();

            drawArmLine(target, radius, "#4da6ff", 4);
            drawArmLine(current, radius, "#39d98a", 9);

            context.fillStyle = "#ffffff";
            context.beginPath();
            context.arc(center.x, center.y, 8, 0, kTwoPi);
            context.fill();

            context.fillStyle = "#d8d8d8";
            context.font = "13px Segoe UI, sans-serif";
            context.fillText("drag target around arc", 26, height - 22);

            function drawArmLine(angle, length, color, widthToUse) {
                context.strokeStyle = color;
                context.lineWidth = widthToUse;
                context.lineCap = "round";
                context.beginPath();
                context.moveTo(center.x, center.y);
                context.lineTo(center.x + Math.cos(angle) * length, center.y - Math.sin(angle) * length);
                context.stroke();
                context.lineCap = "butt";
            }
        }

        function drawFlywheel(width, height) {
            const center = armCenter(width, height);
            const radius = 58;
            const spin = (performance.now() / 180) * Math.max(0.2, sim.output / 30);

            context.strokeStyle = "#39d98a";
            context.lineWidth = 8;
            context.beginPath();
            context.arc(center.x, center.y, radius, 0, kTwoPi);
            context.stroke();

            context.strokeStyle = "#ffffff";
            context.lineWidth = 3;
            for (let i = 0; i < 6; i += 1) {
                const angle = spin + i * Math.PI / 3;
                context.beginPath();
                context.moveTo(center.x, center.y);
                context.lineTo(center.x + Math.cos(angle) * radius, center.y + Math.sin(angle) * radius);
                context.stroke();
            }

            context.fillStyle = "#d8d8d8";
            context.font = "13px Segoe UI, sans-serif";
            context.fillText("click graph height to change speed", 24, height - 22);
        }

        function drawGraph(width, height) {
            const graph = graphRect(width, height);
            const maxValue = currentMode().maxTarget;
            const zero = graph.y + graph.height;

            context.strokeStyle = "#303030";
            context.lineWidth = 1;
            context.strokeRect(graph.x, graph.y, graph.width, graph.height);

            for (let i = 1; i < 4; i += 1) {
                const lineY = graph.y + (graph.height / 4) * i;
                context.beginPath();
                context.moveTo(graph.x, lineY);
                context.lineTo(graph.x + graph.width, lineY);
                context.stroke();
            }

            drawTrace("setpoint", "#4da6ff", maxValue);
            drawTrace("output", "#39d98a", maxValue);

            context.fillStyle = "#d8d8d8";
            context.font = "13px Segoe UI, sans-serif";
            context.fillText("blue: setpoint", graph.x, height - 58);
            context.fillStyle = "#39d98a";
            context.fillText("green: output", graph.x + 112, height - 58);
            context.fillStyle = "#f39c12";
            context.fillText(`voltage ${format(sim.voltage)} V`, graph.x + 226, height - 58);

            function drawTrace(key, color, max) {
                if (sim.history.length < 2) {
                    return;
                }
                context.strokeStyle = color;
                context.lineWidth = 2;
                context.beginPath();
                sim.history.forEach((point, index) => {
                    const x = graph.x + (index / 259) * graph.width;
                    const normalized = sim.mode === "arm"
                        ? (point[key] - currentMode().minTarget) / (max - currentMode().minTarget)
                        : point[key] / max;
                    const y = zero - clamp(normalized, 0, 1) * graph.height;
                    if (index === 0) {
                        context.moveTo(x, y);
                    } else {
                        context.lineTo(x, y);
                    }
                });
                context.stroke();
            }
        }

        function updateReadouts() {
            const unit = currentMode().targetUnit;
            const error = sim.setpoint - sim.output;
            root.querySelector("[data-readout='setpoint']").textContent = `${format(sim.setpoint)} ${unit}`;
            root.querySelector("[data-readout='output']").textContent = `${format(sim.output)} ${unit}`;
            root.querySelector("[data-readout='error']").textContent = `${format(error)} ${unit}`;
            root.querySelector("[data-readout='voltage']").textContent = `${format(sim.voltage)} V`;
            root.querySelector("[data-readout='status']").textContent = statusText(error);
            root.querySelector("[data-lesson]").textContent = lessonText();
        }

        function statusText(error) {
            const settledError = sim.mode === "arm" ? 1.5 : 2.5;
            const settledVelocity = sim.mode === "arm" ? 5 : 20;
            if (Math.abs(error) < settledError && Math.abs(sim.velocity) < settledVelocity) {
                return "Settled";
            }
            if (sim.overshootTimer > 0) {
                return "Overshoot";
            }
            if (Math.abs(sim.voltage) > 11.6) {
                return "Saturated";
            }
            if (sim.strategy === "feedforward" && Math.abs(error) > 8) {
                return "No correction";
            }
            return "Tracking";
        }

        function lessonText() {
            if (sim.mode === "flywheel" && sim.strategy === "feedforward") {
                return "FlywheelSim is a one-state linear system. Feedforward can predict steady voltage, but a shot disturbance still needs feedback to recover.";
            }
            if (sim.mode === "arm" && sim.strategy === "feedforward") {
                return "SingleJointedArmSim adds a gravity acceleration based on cos(angle). Arm feedforward uses the matching cos(angle) voltage term.";
            }
            if (sim.strategy === "feedback") {
                return "The PID math mirrors WPILib PIDController: error, derivative, clamped integral, then kP*error + kI*sum + kD*derivative.";
            }
            return "Combined control is the normal FRC goal: feedforward does the predictable work, PID fixes the leftover error.";
        }

        function animate(now) {
            const elapsed = Math.min((now - sim.lastTime) / 1000, 0.06);
            sim.lastTime = now;

            if (sim.running) {
                let remaining = elapsed;
                while (remaining > 0) {
                    const dt = Math.min(kDtSeconds, remaining);
                    runController(dt);
                    remaining -= dt;
                }
            }

            draw();
            requestAnimationFrame(animate);
        }

        configureControlRanges();
        applyPreset(config.defaultPreset, true);
        updateActiveButtons();
        resetState();
        requestAnimationFrame(animate);
    }

    document.addEventListener("DOMContentLoaded", () => {
        document.querySelectorAll("[data-pid-simulation]").forEach(buildSimulation);
    });
})();
