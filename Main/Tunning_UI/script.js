var connction = new WebSocket("ws://" + location.hostname + ":81/");
let roll = 500,
  pitch = 500,
  throttle = 1000,
  yaw = 500;
let throttleUpdated = false;
let defaultThrottle = 1000,
  defaultYaw = 0,
  defaultRoll = 0,
  defaultPitch = 0;

connction.onopen = () => {
  console.log("WebSocket connection established");
  connction.send(
    JSON.stringify({
      action: "JoystickData",
      throttle: "1000",
      pitch: "0",
      roll: "0",
      yaw: "0",
    })
  );
};

connction.onmessage = (event) => {
  if (event.data == " " || event.data == undefined) {
    console.log("Received an empty or undefined data");
    return;
  }
  const data = JSON.parse(event.data);
  document.getElementById("rollValue").textContent = data["roll"];
  document.getElementById("pitchValue").textContent = data["pitch"];
  document.getElementById("throttleValue").textContent = data["throttle"];
  document.getElementById("yawValue").textContent = data["yaw"];
  document.getElementById("m1Speed").textContent = data["m1"];
  document.getElementById("m2Speed").textContent = data["m2"];
  document.getElementById("m3Speed").textContent = data["m3"];
  document.getElementById("m4Speed").textContent = data["m4"];
  throttleUpdated = true;
};
function setupJoystick(joystickId, handleId, callback, autoResetX, autoResetY) {
  const joystick = document.getElementById(joystickId);
  const handle = document.getElementById(handleId);
  let isDragging = false;

  joystick.addEventListener("mousedown", () => {
    isDragging = true;
    handle.style.cursor = "grabbing";
  });

  document.addEventListener("mouseup", () => {
    if (!isDragging) return;
    isDragging = false;
    handle.style.cursor = "grab";
    resetJoystick(); // Reset only specified axis
  });

  document.addEventListener("mousemove", (e) => {
    if (!isDragging) return;
    const rect = joystick.getBoundingClientRect();
    let x = e.clientX - rect.left;
    let y = e.clientY - rect.top;

    const centerX = joystick.offsetWidth / 2;
    const centerY = joystick.offsetHeight / 2;
    let deltaX = x - centerX;
    let deltaY = y - centerY;

    const maxDistance = joystick.offsetWidth / 2 - handle.offsetWidth / 2;
    const distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

    // Lock movement to one axis at a time
    if (Math.abs(deltaX) > Math.abs(deltaY)) {
      deltaY = 0; // Lock vertical movement
      if (joystickId === "joystickPitchRoll") pitch = defaultPitch; // Reset Pitch when moving Roll
    } else {
      deltaX = 0; // Lock horizontal movement
      if (joystickId === "joystickPitchRoll") roll = defaultRoll; // Reset Roll when moving Pitch
    }

    if (distance > maxDistance) {
      deltaX = (deltaX / distance) * maxDistance;
      deltaY = (deltaY / distance) * maxDistance;
    }

    handle.style.left = `${centerX + deltaX}px`;
    handle.style.top = `${centerY + deltaY}px`;

    callback(deltaX, deltaY, maxDistance);
  });

  function resetJoystick() {
    if (autoResetX) {
      handle.style.left = "50%";
      if (joystickId === "joystickThrottleYaw") yaw = defaultYaw; // Reset Yaw
      if (joystickId === "joystickPitchRoll") roll = defaultRoll; // Reset Roll
    }
    if (autoResetY) {
      handle.style.top = "50%";
      if (joystickId === "joystickPitchRoll") pitch = defaultPitch; // Reset Pitch
    }
    updateReadout();
  }
}

setupJoystick(
  "joystickThrottleYaw",
  "handleThrottleYaw",
  (dx, dy, maxDist) => {
    if (dx !== 0) {
      yaw = Math.round(defaultYaw + (dx / maxDist) * 500);
    }
    if (dy !== 0) {
      // ✅ Map throttle from lowest position (dy = maxDist) to 0, and highest (dy = -maxDist) to 2000
      throttle = Math.round(2000 - ((dy + maxDist) / (2 * maxDist)) * 2000);
    }
  },
  true, // ✅ Auto-reset X (Yaw)
  false // ❌ Do NOT auto-reset Y (Throttle)
);

// Setup Right Joystick: Controls **Pitch (Y) & Roll (X)**
setupJoystick(
  "joystickPitchRoll",
  "handlePitchRoll",
  (dx, dy, maxDist) => {
    if (dx !== 0) {
      roll = Math.round(defaultRoll + (dx / maxDist) * 500);
      pitch = defaultPitch; // ✅ Reset Pitch when moving Roll
    }
    if (dy !== 0) {
      pitch = Math.round(defaultPitch - (dy / maxDist) * 500);
      roll = defaultRoll; // ✅ Reset Roll when moving Pitch
    }
  },
  true, // ✅ Auto-reset X (Roll)
  true // ✅ Auto-reset Y (Pitch)
);
document.getElementById("killSwitch").addEventListener("change", function () {
  if (this.checked) {
    fetch("/kill")
      .then(() => {
        document.getElementById("m1Speed").textContent = "0";
        document.getElementById("m2Speed").textContent = "0";
        document.getElementById("m3Speed").textContent = "0";
        document.getElementById("m4Speed").textContent = "0";
      })
      .catch((error) => console.error("Error sending kill command:", error));
  }
});
// Function to update UI
function sendJoystickData() {
  connction.send(
    JSON.stringify({
      action: "JoystickData",
      throttle: throttle,
      pitch: pitch,
      roll: roll,
      yaw: yaw,
    })
  );
}

// Continuously send throttle data every 100ms if updated
setInterval(() => {
  if (throttleUpdated) {
    sendJoystickData();
    throttleUpdated = false;
    console.log(throttle);
  }
}, 100);
