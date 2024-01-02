const CAR_LONG = 45.0;
const CAR_WIDE = 25.0;
const MAX_SPEED = 35.0;
const CURSOR_RADIUS = 5.0;

const CANVAS_ELEMENT_ID = "mpc";
const CANVAS_WIDTH = document.getElementById(CANVAS_ELEMENT_ID).width;
const CANVAS_HEIGHT = document.getElementById(CANVAS_ELEMENT_ID).height;

const PI = Math.PI;
const HALF_PI = PI / 2.0;
const TWO_PI = PI * 2.0;

const DT = 0.1;

let stopSimulation = false;

function randomInRange(min, max) {
    return Math.random() * (max - min) + min;
}

function fmtNumber(num) {
    num = (Math.round(num * 100.0) / 100.0).toFixed(2);

    if (num >= 0.0) {
        return `+${num}`;
    } else {
        return num;
    }
}

function normalizeRadians(radians) {
    while (radians > PI) radians -= TWO_PI;
    while (radians < -PI) radians += TWO_PI;

    return radians;
}

function draw(car) {
    const canvas = document.getElementById(CANVAS_ELEMENT_ID);
    ctx = canvas.getContext("2d");

    ctx.clearRect(0, 0, canvas.width, canvas.height);

    ctx.fillStyle = "rgb(200, 0, 0)";
    ctx.beginPath();

    const carDirX = Math.cos(car.yaw);
    const carDirY = Math.sin(car.yaw);

    const carPerpX = Math.cos(car.yaw + HALF_PI);
    const carPerpY = Math.sin(car.yaw + HALF_PI);

    const rearX = car.x - carDirX * CAR_LONG / 2.0;
    const rearY = car.y - carDirY * CAR_LONG / 2.0;

    const rearLeftX = rearX + carPerpX * CAR_WIDE / 2.0;
    const rearLeftY = rearY + carPerpY * CAR_WIDE / 2.0;

    const rearRightX = rearX - carPerpX * CAR_WIDE / 2.0;
    const rearRightY = rearY - carPerpY * CAR_WIDE / 2.0;

    const frontX = car.x + carDirX * CAR_LONG / 2.0;
    const frontY = car.y + carDirY * CAR_LONG / 2.0;

    ctx.moveTo(rearLeftX, rearLeftY);
    ctx.lineTo(rearRightX, rearRightY);
    ctx.lineTo(frontX, frontY);
    ctx.closePath();
    ctx.fill();

    ctx.fillStyle = "blue";
    ctx.beginPath();
    ctx.arc(targetX, targetY, CURSOR_RADIUS, 0.0, TWO_PI, false);
    ctx.closePath();
    ctx.fill();

    ctx.fillStyle = "blue";
    ctx.beginPath();
    ctx.arc(spline[car.splineIndex].x, spline[car.splineIndex].y, CURSOR_RADIUS, 0.0, TWO_PI, false);
    ctx.closePath();
    ctx.fill();
}

function drawSpline(spline) {
    ctx.strokeStyle = "rgb(250, 200, 80)";
    ctx.beginPath();

    for (const point of spline) {
        ctx.lineTo(point.x, point.y);
    }

    ctx.closePath();
    ctx.stroke();
}

function maintainInBounds(car) {
    /* Should the model be aware of this? */
    while (car.x > CANVAS_WIDTH) car.x -= CANVAS_WIDTH;
    while (car.x < 0.0) car.x += CANVAS_WIDTH;

    while (car.y < 0.0) car.y += CANVAS_HEIGHT;
    while (car.y > CANVAS_HEIGHT) car.y -= CANVAS_HEIGHT;

    if (car.speed < 0.0) car.speed = 0.0;
    if (car.speed > MAX_SPEED) car.speed = MAX_SPEED;

    car.yaw = normalizeRadians(car.yaw);
}

function frame(car, mpc) {
    if (!stopSimulation) {
        step();
    }

    draw(car);
    drawSpline(spline);

    document.getElementById("table-x").textContent = fmtNumber(car.x);
    document.getElementById("table-y").textContent = fmtNumber(car.y);
    document.getElementById("table-speed").textContent = fmtNumber(car.speed);
    document.getElementById("table-yaw").textContent = fmtNumber(car.yaw);
}

function step() {
    let bestCmd = mpc.predict(car);
    car.update(bestCmd);

    document.getElementById("table-acceleration").textContent = fmtNumber(bestCmd.acceleration);
    document.getElementById("table-steering").textContent = fmtNumber(bestCmd.steering);
}

function resetCarState() {
    car = {
        x: 75.0,
        y: 75.0,
        speed: 0.0,
        yaw: 0.0,
        splineIndex: 15,

        /* Point mass */
        update: function(cmd) {
            const vx = Math.cos(this.yaw) * this.speed;
            const vy = Math.sin(this.yaw) * this.speed;

            this.x += vx * DT;
            this.y += vy * DT;

            this.yaw += cmd.steering * DT;
            this.speed += cmd.acceleration * DT;

            maintainInBounds(this);

            const target = spline[this.splineIndex];
            const dist = (target.x - this.x) ** 2 + (target.y - this.y) ** 2;
            if (dist < 1500)
                this.splineIndex = (this.splineIndex + 1) % spline.length;
            document.getElementById("table-debug").textContent = fmtNumber(this.splineIndex);
        },

        clone: function() {
            return Object.assign({}, this);
        },
    };
}

let car, spline, mpc;
let ctx;

let targetX = 0.0, targetY = 0.0;
window.onmousemove = (e) => {
    const canvas = document.getElementById("mpc");
    if (e.target !== canvas)
        return;
    const rect = canvas.getBoundingClientRect();
    const scaleX = canvas.width / rect.width;
    const scaleY = canvas.height / rect.height;
    targetX = e.offsetX * scaleX;
    targetY = e.offsetY * scaleY;
};

window.addEventListener("load", () => {
    resetCarState();
    setTrackToRing();

    mpc = {
        horizon: 15,
        /* this must have a better name */
        tries: 25,

        calculateCostFor: function(car) {
            const target = spline[car.splineIndex];
            // const correctPositionCost = 1.0 / (
            //     Math.exp(-Math.abs(target.x - car.x)) +
            //     Math.exp(-Math.abs(target.y - car.y))
            // );

            const cost = Math.abs(target.x - car.x) + Math.abs(target.y - car.y);

            // const angle = Math.atan2(target.y - car.y, target.x - car.x);
            // const incentiveToTurn = Math.abs(normalizeRadians(angle - car.yaw));
            // const cost = correctPositionCost; + incentiveToTurn;

            return cost;
        },

        predict: function(car_) {
            const horizon = this.horizon;
            const calculateCostFor = this.calculateCostFor;

            let bestCostSoFar = Infinity;
            let bestCmdSoFar = {};
            let doesBestCmdAdvanceSpline = false;

            function tryCmd(cmd) {
                let car = car_.clone();
                let cost = Infinity;
                let previousSplineIdx = car.splineIndex;

                for (let j = 0; j < horizon; j++) {
                    car.update(cmd);
                    cost = Math.min(cost, calculateCostFor(car));
                }

                let doesCmdAdvanceSpline = previousSplineIdx != car.splineIndex;

                if (bestCostSoFar > cost || (doesCmdAdvanceSpline && !doesBestCmdAdvanceSpline)) {
                    bestCostSoFar = cost;
                    bestCmdSoFar = cmd;
                }
            }

            tryCmd({ acceleration: -5.0, steering: 0.0, });
            tryCmd({ acceleration: +5.0, steering: 0.0, });
            tryCmd({ acceleration: +1.0, steering: -1.0, });
            tryCmd({ acceleration: +1.0, steering: 1.0, });

            for (let i = 0; i < this.tries; i++) {
                const cmd = {
                    acceleration: randomInRange(-5.0, 5.0),
                    steering: randomInRange(-.5, .5),
                };
                tryCmd(cmd);
            }

            document.getElementById("table-cost").textContent = fmtNumber(bestCostSoFar);

            return bestCmdSoFar;
        }
    };

    const trackSelector = document.getElementById("track-select");
    trackSelector.addEventListener("change", function() {
        console.log("ASDFGHJK");

        resetCarState();

        if (trackSelector.value === "acceleration") {
            setTrackToAcceleration();
        } else if (trackSelector.value === "ring") {
            setTrackToRing();
        }
    });

    loop();
});

function setTrackToAcceleration() {
    const LENGTH = 700.0;

    const START_X = 100.0;
    const START_Y = 175.0;

    car.x = START_X;
    car.y = START_Y;

    spline = [];
    for (let i = 0; i < 20; i++) {

        spline.push({ x: START_X + LENGTH * i / 20, y: START_Y });
    }
}

function setTrackToRing() {
    const WIDTH = 300.0;
    const HEIGHT = 150.0;

    const START_X = 400.0;
    const START_Y = 175.0;

    spline = [];
    for (let i = 0; i < 20; i++) {

        const v = i / 20 * Math.PI * 2.0;
        spline.push({ x: START_X + Math.cos(v) * WIDTH, y: START_Y + Math.sin(v) * HEIGHT });
    }
}

function loop(dt) {
    frame(car, mpc);
    requestAnimationFrame(loop);
}
