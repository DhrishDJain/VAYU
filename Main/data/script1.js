var connction = new WebSocket("ws://" + location.hostname + ":81/");
let roll = 0,
  pitch = 0,
  throttle = 1000,
  yaw = 0;
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

class CameraStream {
  constructor() {
    this.container = document.getElementById("mjpeg-container");
    this.statusDot = document.querySelector(".status-dot");
    this.statusText = document.querySelector(".status-text");
    this.toggleButton = document.getElementById("stream-toggle");
    this.isStreaming = false;
    this.imgElement = null;
  }

  init() {
    this.toggleButton.addEventListener("click", () => this.toggleStream());
    this.startStream();
  }

  toggleStream() {
    if (this.isStreaming) {
      this.stopStream();
    } else {
      this.startStream();
    }
  }

  startStream() {
    this.isStreaming = true;
    this.updateStatus("Connecting...", "#ffa502");

    if (!this.imgElement) {
      this.imgElement = document.createElement("img");
      this.imgElement.className = "mjpeg-stream";
      this.container.appendChild(this.imgElement);
    }

    this.imgElement.src = `http://192.168.0.18/stream?t=${Date.now()}`;

    this.imgElement.onload = () => {
      this.updateStatus("Live", "#2ed573");
      this.statusDot.classList.add("active");
    };

    this.imgElement.onerror = () => {
      this.updateStatus("Connection Error", "#ff4757");
      this.statusDot.classList.remove("active");
      setTimeout(() => this.startStream(), 2000);
    };
  }

  stopStream() {
    this.isStreaming = false;
    if (this.imgElement) {
      this.imgElement.src = "";
      this.updateStatus("Stream Stopped", "#ff4757");
      this.statusDot.classList.remove("active");
    }
  }

  updateStatus(text, color) {
    this.statusText.textContent = text;
    this.statusDot.style.backgroundColor = color;
  }
}

document.addEventListener("DOMContentLoaded", () => {
  const leftJoystick = document.getElementById("joystickThrottleYaw");
  const rightJoystick = document.getElementById("joystickPitchRoll");
  let cameraActive = false;
  const cameraToggle = document.querySelector(".camera-toggle");
  const cameraFeed = document.querySelector(".camera-feed");
  const feedText = document.querySelector(".camera-feed-text");
  const throttleHandle = document.getElementById("handleThrottleYaw");
  throttleHandle.style.top = 80 + "%";
  cameraToggle.addEventListener("click", () => {
    cameraActive = !cameraActive;
    cameraFeed.classList.toggle("active", cameraActive);
    feedText.style.display = cameraActive ? "none" : "block";
  });

  function setupJoystick(
    joystickId,
    handleId,
    callback,
    autoResetX,
    autoResetY
  ) {
    const joystick = document.getElementById(joystickId);
    const handle = document.getElementById(handleId);
    let isDragging = false;

    joystick.addEventListener("pointerdown", () => {
      isDragging = true;
      handle.style.cursor = "grabbing";
    });

    document.addEventListener("pointerup", () => {
      if (!isDragging) return;
      isDragging = false;
      handle.style.cursor = "grab";
      resetJoystick(); // Reset only specified axis
    });

    document.addEventListener("pointermove", (e) => {
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
      console.log(deltaY);
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
        throttle = Math.round(1000 + ((maxDist - dy) / (2 * maxDist)) * 1000);
      }
      // console.log(throttle);
      updateReadout(); // Update the readout with new values
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
        roll = Math.round((dx / maxDist) * 100); // Map -100 to 100
      }
      if (dy !== 0) {
        pitch = Math.round((dy / maxDist) * 100); // Map -100 to 100
      }
      updateReadout(); // Update the readout with new values
    },
    true, // ✅ Auto-reset X (Roll)
    true // ✅ Auto-reset Y (Pitch)
  );

  function updateReadout() {
    const throt = (((throttle - 1000) / 1000) * 100).toFixed(0);
    document.querySelector(".throttleValue").textContent = `${throt}%`;
  }

  // Function to send the throttle value to the server

  setInterval(function () {
    // sendThrottleValue(throttle);
    if (connction.readyState == 1) {
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
  }, 100);

  // battery drain
  let battery = 87;
  const battDisp = document.querySelector(".battery-level");
  setInterval(() => {
    battery = battery < 0 ? 100 : battery - 0.1;
    const lvl = Math.round(battery);
    battDisp.textContent = `${lvl}%`;
    battDisp.className =
      "reading-value battery-level" +
      (lvl < 20 ? " critical" : lvl < 40 ? " warning" : "");
  }, 3000);

  // random signal bars
  setInterval(() => {
    const bars = document.querySelectorAll(".signal-bar");
    const active = Math.floor(Math.random() * 2) + 3;
    bars.forEach((b, i) => (b.style.opacity = i < active ? "1" : "0.3"));
  }, 5000);

  // reset on resize
  window.addEventListener("resize", () => {
    [leftJoystick, rightJoystick].forEach((j) => {
      j.style.transform = "translate(-50%,-50%)";
    });
  });
});
