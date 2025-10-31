from http.server import BaseHTTPRequestHandler, HTTPServer
import json
import threading
import textwrap
from typing import Any, Dict, List, Tuple, Type

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from dsr_msgs2.srv import DrlStart, Fkin, GetCurrentPosj, MoveJoint, MoveLine, MoveStop, SetRobotMode


HTML_PAGE = """<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="utf-8">
    <title>Joint & Line Motion Controller</title>
    <style>
        body { font-family: Arial, sans-serif; margin: 28px; background-color: #f5f5f5; }
        h1 { margin-bottom: 24px; }
        .forms { display: flex; flex-direction: column; gap: 28px; max-width: 760px; }
        section { background: #ffffff; border: 1px solid #d0d0d0; border-radius: 8px; padding: 20px; box-shadow: 0 2px 4px rgba(0,0,0,0.06); }
        section h2 { margin-top: 0; margin-bottom: 16px; font-size: 1.2rem; }
        form { display: grid; grid-template-columns: repeat(auto-fit, minmax(160px, 1fr)); gap: 8px 12px; }
        label { display: flex; flex-direction: column; font-weight: bold; font-size: 0.9rem; }
        label span { font-weight: normal; font-size: 0.75rem; color: #555; }
        input[type=number], input[type=text] { padding: 6px; font-size: 1rem; }
        select { padding: 6px; font-size: 1rem; }
        .sign-input { display: flex; align-items: center; gap: 6px; }
        .sign-input button { padding: 4px 10px; font-size: 0.95rem; cursor: pointer; border: 1px solid #bbb; background: #f4f4f4; border-radius: 4px; }
        .sign-input button:active { background: #e0e0e0; }
        .sign-input input { flex: 1 1 50%; min-width: 0; padding: 6px; font-size: 1rem; }
        button { grid-column: span 2; padding: 10px; font-size: 1rem; cursor: pointer; }
        .status { margin-top: 16px; min-height: 1.2em; font-weight: bold; }
        .result { margin-top: 8px; min-height: 1.2em; font-family: "Courier New", monospace; font-size: 0.9rem; color: #333; }
        .success { color: #1b5e20; }
        .error { color: #b71c1c; }
        .help { grid-column: span 2; font-size: 0.8rem; color: #666; }
        .estop-button { margin-top: 18px; padding: 14px 20px; font-size: 1.05rem; font-weight: bold; color: #fff; background: #d32f2f; border: none; border-radius: 6px; cursor: pointer; }
        .estop-button:active { background: #b71c1c; }
        @media (max-width: 520px) {
            button, .help { grid-column: span 1; }
        }
    </style>
</head>
<body>
    <h1>Doosan Motion Controller</h1>
    <div class="forms">
        <section>
            <h2>Move Joint</h2>
            <form id="moveJointForm">
                <label>Joint 1 (deg)<input type="text" inputmode="decimal" pattern="-?[0-9]*([.][0-9]+)?" name="j1" value="0.0" data-signable="true"></label>
                <label>Joint 2 (deg)<input type="text" inputmode="decimal" pattern="-?[0-9]*([.][0-9]+)?" name="j2" value="0.0" data-signable="true"></label>
                <label>Joint 3 (deg)<input type="text" inputmode="decimal" pattern="-?[0-9]*([.][0-9]+)?" name="j3" value="90.0" data-signable="true"></label>
                <label>Joint 4 (deg)<input type="text" inputmode="decimal" pattern="-?[0-9]*([.][0-9]+)?" name="j4" value="0.0" data-signable="true"></label>
                <label>Joint 5 (deg)<input type="text" inputmode="decimal" pattern="-?[0-9]*([.][0-9]+)?" name="j5" value="90.0" data-signable="true"></label>
                <label>Joint 6 (deg)<input type="text" inputmode="decimal" pattern="-?[0-9]*([.][0-9]+)?" name="j6" value="0.0" data-signable="true"></label>
                <label>Velocity (deg/s)<input type="text" inputmode="decimal" pattern="-?[0-9]*([.][0-9]+)?" name="vel" value="20.0" data-signable="true"></label>
                <label>Acceleration (deg/sÂ²)<input type="text" inputmode="decimal" pattern="-?[0-9]*([.][0-9]+)?" name="acc" value="40.0" data-signable="true"></label>
                <label>Time (s)<input type="text" inputmode="decimal" pattern="-?[0-9]*([.][0-9]+)?" name="time" value="0.0" data-signable="true"></label>
                <label>Radius (mm)<input type="text" inputmode="decimal" pattern="-?[0-9]*([.][0-9]+)?" name="radius" value="0.0" data-signable="true"></label>
                <label>Mode<input type="text" inputmode="numeric" pattern="-?[0-9]+" name="mode" value="0" data-signable="true"><span>0=ABS, 1=REL</span></label>
                <label>Blend Type<input type="text" inputmode="numeric" pattern="-?[0-9]+" name="blend_type" value="0" data-signable="true"></label>
                <label>Sync Type<input type="text" inputmode="numeric" pattern="-?[0-9]+" name="sync_type" value="0" data-signable="true"></label>
                <button type="submit">Send MoveJoint</button>
                <div class="help">Service: /dsr01/motion/move_joint</div>
            </form>
            <div class="status" id="moveJointStatus"></div>
            <div class="result" id="moveJointResult"></div>
        </section>
        <section>
            <h2>Move Line</h2>
            <form id="moveLineForm">
                <label>Pos 1 (mm)<input type="text" inputmode="decimal" pattern="-?[0-9]*([.][0-9]+)?" name="p1" value="400.0" data-signable="true"></label>
                <label>Pos 2 (mm)<input type="text" inputmode="decimal" pattern="-?[0-9]*([.][0-9]+)?" name="p2" value="-100.0" data-signable="true"></label>
                <label>Pos 3 (mm)<input type="text" inputmode="decimal" pattern="-?[0-9]*([.][0-9]+)?" name="p3" value="300.0" data-signable="true"></label>
                <label>Pos 4 (deg)<input type="text" inputmode="decimal" pattern="-?[0-9]*([.][0-9]+)?" name="p4" value="0.0" data-signable="true"></label>
                <label>Pos 5 (deg)<input type="text" inputmode="decimal" pattern="-?[0-9]*([.][0-9]+)?" name="p5" value="180.0" data-signable="true"></label>
                <label>Pos 6 (deg)<input type="text" inputmode="decimal" pattern="-?[0-9]*([.][0-9]+)?" name="p6" value="0.0" data-signable="true"></label>
                <label>Velocity Linear (mm/s)<input type="text" inputmode="decimal" pattern="-?[0-9]*([.][0-9]+)?" name="vel_lin" value="20.0" data-signable="true"></label>
                <label>Velocity Angular (deg/s)<input type="text" inputmode="decimal" pattern="-?[0-9]*([.][0-9]+)?" name="vel_ang" value="40.0" data-signable="true"></label>
                <label>Acceleration Linear (mm/sÂ²)<input type="text" inputmode="decimal" pattern="-?[0-9]*([.][0-9]+)?" name="acc_lin" value="20.0" data-signable="true"></label>
                <label>Acceleration Angular (deg/sÂ²)<input type="text" inputmode="decimal" pattern="-?[0-9]*([.][0-9]+)?" name="acc_ang" value="40.0" data-signable="true"></label>
                <label>Time (s)<input type="text" inputmode="decimal" pattern="-?[0-9]*([.][0-9]+)?" name="time" value="0.0" data-signable="true"></label>
                <label>Radius (mm)<input type="text" inputmode="decimal" pattern="-?[0-9]*([.][0-9]+)?" name="radius" value="0.0" data-signable="true"></label>
                <label>Reference<input type="text" inputmode="numeric" pattern="-?[0-9]+" name="ref" value="0" data-signable="true"><span>0=BASE, 1=TOOL, 2=WORLD</span></label>
                <label>Mode<input type="text" inputmode="numeric" pattern="-?[0-9]+" name="mode" value="0" data-signable="true"><span>0=ABS, 1=REL</span></label>
                <label>Blend Type<input type="text" inputmode="numeric" pattern="-?[0-9]+" name="blend_type" value="0" data-signable="true"></label>
                <label>Sync Type<input type="text" inputmode="numeric" pattern="-?[0-9]+" name="sync_type" value="0" data-signable="true"></label>
                <button type="submit">Send MoveLine</button>
                <div class="help">Service: /dsr01/motion/move_line</div>
            </form>
            <div class="status" id="moveLineStatus"></div>
            <div class="result" id="moveLineResult"></div>
        </section>
        <section>
            <h2>Gripper Control</h2>
            <form id="gripperForm">
                <label>Command<select name="command">
                    <option value="initialize">Initialize</option>
                    <option value="move" selected>Move</option>
                    <option value="terminate">Terminate</option>
                </select></label>
                <label>Stroke (0-700)<input type="number" name="stroke" min="0" max="700" step="10" value="700" required>
                    <span>Used when Command is Move</span>
                </label>
                <button type="submit">Send Gripper Command</button>
                <div class="help">Service: /dsr01/drl/drl_start</div>
            </form>
            <div class="status" id="gripperStatus"></div>
            <div class="result" id="gripperResult"></div>
        </section>
        <section>
            <h2>Robot Mode Control</h2>
            <form id="robotModeForm">
                <label>Robot Mode<select name="robot_mode">
                    <option value="0">Manual (0)</option>
                    <option value="1">Autonomous (1)</option>
                    <option value="2">Measure (2)</option>
                </select></label>
                <button type="submit">Apply Robot Mode</button>
                <div class="help">Service: /dsr01/system/set_robot_mode</div>
            </form>
            <div class="status" id="robotModeStatus"></div>
            <button id="estopButton" class="estop-button">Emergency Stop (Quick)</button>
            <div class="status" id="estopStatus"></div>
        </section>
    </div>
    <script>
        function setupSignControls() {
            const inputs = document.querySelectorAll('input[data-signable="true"]');
            inputs.forEach((input) => {
                const wrapper = document.createElement('span');
                wrapper.className = 'sign-input';

                const minusBtn = document.createElement('button');
                minusBtn.type = 'button';
                minusBtn.textContent = 'âˆ’';

                const applyNegativeSign = () => {
                    const current = Number.parseFloat(input.value);
                    if (Number.isNaN(current)) {
                        input.value = '-0.0';
                        return;
                    }
                    const magnitude = Math.abs(current);
                    const signedValue = -magnitude;
                    input.value = signedValue.toString();
                };

                minusBtn.addEventListener('click', (event) => {
                    event.preventDefault();
                    applyNegativeSign();
                });

                const parent = input.parentNode;
                if (parent) {
                    parent.replaceChild(wrapper, input);
                    wrapper.appendChild(minusBtn);
                    wrapper.appendChild(input);
                }
            });
        }

        function formatValues(values) {
            if (!Array.isArray(values)) {
                return '';
            }
            return values.map((value) => {
                const num = Number.parseFloat(value);
                return Number.isFinite(num) ? num.toFixed(3) : String(value);
            }).join(', ');
        }

        async function submitForm(form, statusBox, endpoint, payloadBuilder, options = {}) {
            const { resultBox, renderResult, successMessage } = options;
            statusBox.textContent = 'Sending command...';
            statusBox.className = 'status';
            if (resultBox) {
                resultBox.textContent = '';
            }
            try {
                const payload = payloadBuilder(form);
                const response = await fetch(endpoint, {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify(payload)
                });
                const result = await response.json();
                if (response.ok && result.success) {
                    const message = typeof successMessage === 'function'
                        ? successMessage(result)
                        : (successMessage || 'Command sent successfully.');
                    statusBox.textContent = message;
                    statusBox.className = 'status success';
                    if (resultBox) {
                        const rendered = renderResult ? renderResult(result) : '';
                        resultBox.textContent = rendered;
                    }
                } else {
                    const message = (result && result.error) ? result.error : 'Unknown error';
                    statusBox.textContent = `Failed: ${message}`;
                    statusBox.className = 'status error';
                    if (resultBox) {
                        resultBox.textContent = '';
                    }
                }
            } catch (err) {
                statusBox.textContent = `Request error: ${err}`;
                statusBox.className = 'status error';
                if (resultBox) {
                    resultBox.textContent = '';
                }
            }
        }

        setupSignControls();

        document.getElementById('moveJointForm').addEventListener('submit', (event) => {
            event.preventDefault();
            submitForm(
                event.target,
                document.getElementById('moveJointStatus'),
                '/api/move_joint',
                (form) => ({
                    pos: [
                        parseFloat(form.j1.value),
                        parseFloat(form.j2.value),
                        parseFloat(form.j3.value),
                        parseFloat(form.j4.value),
                        parseFloat(form.j5.value),
                        parseFloat(form.j6.value)
                    ],
                    vel: parseFloat(form.vel.value),
                    acc: parseFloat(form.acc.value),
                    time: parseFloat(form.time.value),
                    radius: parseFloat(form.radius.value),
                    mode: parseInt(form.mode.value, 10),
                    blend_type: parseInt(form.blend_type.value, 10),
                    sync_type: parseInt(form.sync_type.value, 10)
                }),
                {
                    resultBox: document.getElementById('moveJointResult'),
                    renderResult: (result) => {
                        if (Array.isArray(result.converted_posx)) {
                            return `Task pose (posx): ${formatValues(result.converted_posx)}`;
                        }
                        if (result.conversion_error) {
                            return `Conversion error: ${result.conversion_error}`;
                        }
                        return '';
                    }
                }
            );
        });

        document.getElementById('moveLineForm').addEventListener('submit', (event) => {
            event.preventDefault();
            submitForm(
                event.target,
                document.getElementById('moveLineStatus'),
                '/api/move_line',
                (form) => ({
                    pos: [
                        parseFloat(form.p1.value),
                        parseFloat(form.p2.value),
                        parseFloat(form.p3.value),
                        parseFloat(form.p4.value),
                        parseFloat(form.p5.value),
                        parseFloat(form.p6.value)
                    ],
                    vel: [
                        parseFloat(form.vel_lin.value),
                        parseFloat(form.vel_ang.value)
                    ],
                    acc: [
                        parseFloat(form.acc_lin.value),
                        parseFloat(form.acc_ang.value)
                    ],
                    time: parseFloat(form.time.value),
                    radius: parseFloat(form.radius.value),
                    ref: parseInt(form.ref.value, 10),
                    mode: parseInt(form.mode.value, 10),
                    blend_type: parseInt(form.blend_type.value, 10),
                    sync_type: parseInt(form.sync_type.value, 10)
                }),
                {
                    resultBox: document.getElementById('moveLineResult'),
                    renderResult: (result) => {
                        if (Array.isArray(result.current_posj)) {
                            return `Current joint pose: ${formatValues(result.current_posj)}`;
                        }
                        if (result.state_error) {
                            return `State read error: ${result.state_error}`;
                        }
                        return '';
                    }
                }
            );
        });

        const gripperForm = document.getElementById('gripperForm');
        if (gripperForm) {
            const commandSelect = gripperForm.elements.command;
            const strokeInput = gripperForm.elements.stroke;
            const strokeLabel = strokeInput.closest('label');

            const updateStrokeAvailability = () => {
                const isMoveCommand = commandSelect.value === 'move';
                strokeInput.disabled = !isMoveCommand;
                if (strokeLabel) {
                    strokeLabel.style.opacity = isMoveCommand ? '1' : '0.55';
                }
            };

            commandSelect.addEventListener('change', updateStrokeAvailability);
            updateStrokeAvailability();

            gripperForm.addEventListener('submit', (event) => {
                event.preventDefault();
                const statusBox = document.getElementById('gripperStatus');
                const resultBox = document.getElementById('gripperResult');
                if (!statusBox) {
                    console.error('Gripper status element is missing.');
                    return;
                }
                const command = commandSelect.value;
                let strokeValue = null;
                if (command === 'move') {
                    const parsedStroke = Number.parseInt(strokeInput.value, 10);
                    if (!Number.isFinite(parsedStroke)) {
                        if (statusBox) {
                            statusBox.textContent = 'Stroke value must be a number.';
                            statusBox.className = 'status error';
                        }
                        if (resultBox) {
                            resultBox.textContent = '';
                        }
                        return;
                    }
                    strokeValue = parsedStroke;
                }

                submitForm(
                    gripperForm,
                    statusBox,
                    '/api/gripper',
                    () => {
                        const payload = { command };
                        if (command === 'move' && strokeValue !== null) {
                            payload.stroke = strokeValue;
                        }
                        return payload;
                    },
                    {
                        resultBox,
                        successMessage: (result) => result.message || 'Command sent successfully.',
                        renderResult: (result) => {
                            if (result.error) {
                                return result.error;
                            }
                            if (result.command === 'move' && typeof result.stroke !== 'undefined') {
                                return `Stroke: ${result.stroke}`;
                            }
                            return result.message || '';
                        }
                    }
                );
            });
        }

        document.getElementById('robotModeForm').addEventListener('submit', (event) => {
            event.preventDefault();
            submitForm(
                event.target,
                document.getElementById('robotModeStatus'),
                '/api/set_robot_mode',
                (form) => ({
                    robot_mode: parseInt(form.robot_mode.value, 10)
                }),
                {
                    successMessage: (result) => result.message || 'Command sent successfully.'
                }
            );
        });

        document.getElementById('estopButton').addEventListener('click', (event) => {
            event.preventDefault();
            submitForm(
                null,
                document.getElementById('estopStatus'),
                '/api/move_stop',
                () => ({ stop_mode: 1 }),
                {
                    successMessage: (result) => result.message || 'Emergency stop requested.'
                }
            );
        });
    </script>
</body>
</html>
"""

GRIPPER_DRL_BASE_CODE = textwrap.dedent("""
g_slaveid = 0
flag = 0
def modbus_set_slaveid(slaveid):
    global g_slaveid
    g_slaveid = slaveid
def modbus_fc06(address, value):
    global g_slaveid
    data = (g_slaveid).to_bytes(1, byteorder='big')
    data += (6).to_bytes(1, byteorder='big')
    data += (address).to_bytes(2, byteorder='big')
    data += (value).to_bytes(2, byteorder='big')
    return modbus_send_make(data)
def modbus_fc16(startaddress, cnt, valuelist):
    global g_slaveid
    data = (g_slaveid).to_bytes(1, byteorder='big')
    data += (16).to_bytes(1, byteorder='big')
    data += (startaddress).to_bytes(2, byteorder='big')
    data += (cnt).to_bytes(2, byteorder='big')
    data += (2 * cnt).to_bytes(1, byteorder='big')
    for i in range(0, cnt):
        data += (valuelist[i]).to_bytes(2, byteorder='big')
    return modbus_send_make(data)
def recv_check():
    size, val = flange_serial_read(0.1)
    if size > 0:
        return True, val
    else:
        tp_log("CRC Check Fail")
        return False, val
def gripper_move(stroke):
    flange_serial_write(modbus_fc16(282, 2, [stroke, 0]))
    wait(1.0)

while True:
    flange_serial_open(
        baudrate=57600,
        bytesize=DR_EIGHTBITS,
        parity=DR_PARITY_NONE,
        stopbits=DR_STOPBITS_ONE,
    )

    modbus_set_slaveid(1)

    flange_serial_write(modbus_fc06(256, 1))
    flag, val = recv_check()

    flange_serial_write(modbus_fc06(275, 400))
    flag, val = recv_check()

    if flag is True:
        break

    flange_serial_close()
""").strip()

GRIPPER_INITIALIZE_SNIPPET = textwrap.dedent("""
flange_serial_open(baudrate=57600, bytesize=DR_EIGHTBITS, parity=DR_PARITY_NONE, stopbits=DR_STOPBITS_ONE)
modbus_set_slaveid(1)
flange_serial_write(modbus_fc06(256, 1))
recv_check()
flange_serial_write(modbus_fc06(275, 400))
recv_check()
""").strip()

GRIPPER_TERMINATE_SNIPPET = "flange_serial_close()"

GRIPPER_STROKE_MIN = 0
GRIPPER_STROKE_MAX = 700


def sanitize_move_joint_payload(payload: Dict[str, Any]) -> MoveJoint.Request:
    required = ['pos', 'vel', 'acc', 'time', 'radius', 'mode', 'blend_type', 'sync_type']
    for key in required:
        if key not in payload:
            raise ValueError(f"Missing field: {key}")

    pos_raw = payload['pos']
    if not isinstance(pos_raw, list) or len(pos_raw) != 6:
        raise ValueError("Field 'pos' must be a list with 6 numeric items.")

    pos: List[float] = [float(value) for value in pos_raw]

    request = MoveJoint.Request()
    request.pos = pos
    request.vel = float(payload['vel'])
    request.acc = float(payload['acc'])
    request.time = float(payload['time'])
    request.radius = float(payload['radius'])
    request.mode = int(payload['mode'])
    request.blend_type = int(payload['blend_type'])
    request.sync_type = int(payload['sync_type'])
    return request


def sanitize_move_line_payload(payload: Dict[str, Any]) -> MoveLine.Request:
    required = ['pos', 'vel', 'acc', 'time', 'radius', 'ref', 'mode', 'blend_type', 'sync_type']
    for key in required:
        if key not in payload:
            raise ValueError(f"Missing field: {key}")

    pos_raw = payload['pos']
    if not isinstance(pos_raw, list) or len(pos_raw) != 6:
        raise ValueError("Field 'pos' must be a list with 6 numeric items.")

    vel_raw = payload['vel']
    if not isinstance(vel_raw, list) or len(vel_raw) != 2:
        raise ValueError("Field 'vel' must be a list with 2 numeric items.")

    acc_raw = payload['acc']
    if not isinstance(acc_raw, list) or len(acc_raw) != 2:
        raise ValueError("Field 'acc' must be a list with 2 numeric items.")

    request = MoveLine.Request()
    request.pos = [float(value) for value in pos_raw]
    request.vel = [float(value) for value in vel_raw]
    request.acc = [float(value) for value in acc_raw]
    request.time = float(payload['time'])
    request.radius = float(payload['radius'])
    request.ref = int(payload['ref'])
    request.mode = int(payload['mode'])
    request.blend_type = int(payload['blend_type'])
    request.sync_type = int(payload['sync_type'])
    return request


class EmergencyStopClient(Node):
    """ë¹„ìƒì •ì§€ ì „ìš© ë…ë¦½ ë…¸ë“œ - ë‹¤ë¥¸ ëª…ë ¹ì–´ì™€ ë…ë¦½ì ìœ¼ë¡œ ì¦‰ì‹œ ì‹¤í–‰"""
    def __init__(self) -> None:
        super().__init__('emergency_stop_client')
        self._move_stop_client = self.create_client(MoveStop, '/dsr01/motion/move_stop')
        self._wait_for_service(self._move_stop_client, '/dsr01/motion/move_stop')

    def move_stop(self, stop_mode: int) -> bool:
        request = MoveStop.Request()
        request.stop_mode = int(stop_mode)
        future = self._move_stop_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() is None:
            self.get_logger().error('Emergency stop service call failed or timed out.')
            return False
        return bool(future.result().success)

    def _wait_for_service(self, client: Any, name: str) -> None:
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Waiting for {name} service...')


class MotionClient(Node):
    def __init__(self) -> None:
        super().__init__('motion_web_client')
        self._joint_client = self.create_client(MoveJoint, '/dsr01/motion/move_joint')
        self._line_client = self.create_client(MoveLine, '/dsr01/motion/move_line')
        self._fkin_client = self.create_client(Fkin, '/dsr01/motion/fkin')
        self._robot_mode_client = self.create_client(SetRobotMode, '/dsr01/system/set_robot_mode')
        self._current_posj_client = self.create_client(GetCurrentPosj, '/dsr01/aux_control/get_current_posj')
        self._drl_client = self.create_client(DrlStart, '/dsr01/drl/drl_start')
        self._gripper_robot_system = 0

        self._wait_for_service(self._joint_client, '/dsr01/motion/move_joint')
        self._wait_for_service(self._line_client, '/dsr01/motion/move_line')
        self._wait_for_service(self._fkin_client, '/dsr01/motion/fkin')
        self._wait_for_service(self._robot_mode_client, '/dsr01/system/set_robot_mode')
        self._wait_for_service(self._current_posj_client, '/dsr01/aux_control/get_current_posj')
        self._wait_for_service(self._drl_client, '/dsr01/drl/drl_start')

    def call_move_joint(self, request: MoveJoint.Request) -> bool:
        response = self._call_service(self._joint_client, request, 'MoveJoint')
        return bool(response.success)

    def call_move_line(self, request: MoveLine.Request) -> bool:
        response = self._call_service(self._line_client, request, 'MoveLine')
        return bool(response.success)

    def convert_joint_to_task(self, joints: List[float], ref: int = 0) -> List[float]:
        request = Fkin.Request()
        request.pos = list(joints)
        request.ref = int(ref)
        response = self._call_service(self._fkin_client, request, 'Fkin')
        if not response.success:
            raise RuntimeError('Fkin conversion failed.')
        return list(response.conv_posx)

    def set_robot_mode(self, mode: int) -> bool:
        request = SetRobotMode.Request()
        request.robot_mode = int(mode)
        response = self._call_service(self._robot_mode_client, request, 'SetRobotMode')
        return bool(response.success)

    def get_current_posj(self) -> List[float]:
        request = GetCurrentPosj.Request()
        response = self._call_service(self._current_posj_client, request, 'GetCurrentPosj')
        if not response.success:
            raise RuntimeError('GetCurrentPosj service reported failure.')
        return list(response.pos)

    def initialize_gripper(self) -> bool:
        script = f"{GRIPPER_DRL_BASE_CODE}\n{GRIPPER_INITIALIZE_SNIPPET}"
        self.get_logger().info('Requesting gripper initialization.')
        return self._send_gripper_script(script)

    def move_gripper(self, stroke: int) -> bool:
        stroke_int = int(stroke)
        if stroke_int < GRIPPER_STROKE_MIN or stroke_int > GRIPPER_STROKE_MAX:
            raise ValueError(f'Stroke must be between {GRIPPER_STROKE_MIN} and {GRIPPER_STROKE_MAX}.')
        move_snippet = textwrap.dedent(f"""
            gripper_move({stroke_int})
        """).strip()
        script = f"{GRIPPER_DRL_BASE_CODE}\n{move_snippet}"
        self.get_logger().info(f'Requesting gripper move to stroke {stroke_int}.')
        return self._send_gripper_script(script)

    def terminate_gripper(self) -> bool:
        self.get_logger().info('Requesting gripper termination.')
        return self._send_gripper_script(GRIPPER_TERMINATE_SNIPPET)

    def _wait_for_service(self, client: Any, name: str) -> None:
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Waiting for {name} service...')

    def _call_service(self, client: Any, request: Any, name: str) -> Any:
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=None)
        if future.result() is None:
            raise RuntimeError(f'{name} service call timed out or failed.')
        return future.result()

    def _send_gripper_script(self, code: str) -> bool:
        request = DrlStart.Request()
        request.robot_system = self._gripper_robot_system
        request.code = code
        response = self._call_service(self._drl_client, request, 'DrlStart')
        return bool(response.success)


class MotionRequestHandler(BaseHTTPRequestHandler):
    server_version = "MotionHTTP/0.3"

    def do_OPTIONS(self) -> None:  # noqa: N802
        if self.path in ('/', '/index.html', '/api/move_joint', '/api/move_line', '/api/gripper', '/api/set_robot_mode', '/api/move_stop'):
            self.send_response(200)
            self._set_cors_headers()
            self.end_headers()
        else:
            self.send_error(404, "Not Found")

    def do_GET(self) -> None:  # noqa: N802
        if self.path in ('/', '/index.html'):
            self._send_html(HTML_PAGE)
        else:
            self.send_error(404, "Not Found")

    def do_POST(self) -> None:  # noqa: N802
        if self.path not in ('/api/move_joint', '/api/move_line', '/api/gripper', '/api/set_robot_mode', '/api/move_stop'):
            self.send_error(404, "Not Found")
            return

        length = int(self.headers.get('Content-Length', 0))
        if length <= 0:
            self._send_json(400, {'error': 'Empty request body.'})
            return

        raw_body = self.rfile.read(length)
        try:
            payload = json.loads(raw_body.decode('utf-8'))
            if self.path == '/api/move_joint':
                request = sanitize_move_joint_payload(payload)
                success = self.server.node.call_move_joint(request)  # type: ignore[attr-defined]
                response_payload: Dict[str, Any] = {'success': success}
                if success:
                    try:
                        converted = self.server.node.convert_joint_to_task(list(request.pos))  # type: ignore[attr-defined]
                        response_payload['converted_posx'] = converted
                    except Exception as conv_err:  # noqa: BLE001
                        response_payload['conversion_error'] = str(conv_err)
                        self.server.node.get_logger().warning(f'Failed to convert posj to posx: {conv_err}')  # type: ignore[attr-defined]
                self._send_json(200, response_payload)
                return
            elif self.path == '/api/move_line':
                request = sanitize_move_line_payload(payload)
                success = self.server.node.call_move_line(request)  # type: ignore[attr-defined]
                response_payload = {'success': success}
                if success:
                    try:
                        current_posj = self.server.node.get_current_posj()  # type: ignore[attr-defined]
                        response_payload['current_posj'] = current_posj
                    except Exception as conv_err:  # noqa: BLE001
                        response_payload['state_error'] = str(conv_err)
                        self.server.node.get_logger().warning(f'Failed to read current posj: {conv_err}')  # type: ignore[attr-defined]
                self._send_json(200, response_payload)
                return
            elif self.path == '/api/gripper':
                if 'command' not in payload:
                    raise ValueError('Missing field: command')
                command = str(payload['command']).lower()
                if command == 'initialize':
                    success = self.server.node.initialize_gripper()  # type: ignore[attr-defined]
                    message = 'Gripper initialization command sent.'
                    response_payload = {'success': success, 'command': command, 'message': message}
                elif command == 'move':
                    if 'stroke' not in payload:
                        raise ValueError('Missing field: stroke')
                    stroke_raw = payload['stroke']
                    stroke_value = int(float(stroke_raw))
                    success = self.server.node.move_gripper(stroke_value)  # type: ignore[attr-defined]
                    message = f'Gripper move command sent (stroke {stroke_value}).'
                    response_payload = {
                        'success': success,
                        'command': command,
                        'stroke': stroke_value,
                        'message': message
                    }
                elif command == 'terminate':
                    success = self.server.node.terminate_gripper()  # type: ignore[attr-defined]
                    message = 'Gripper terminate command sent.'
                    response_payload = {'success': success, 'command': command, 'message': message}
                else:
                    raise ValueError(f'Unsupported gripper command: {command}')
                if not response_payload['success']:
                    response_payload['error'] = 'Gripper service reported failure.'
                self._send_json(200, response_payload)
                return
            elif self.path == '/api/set_robot_mode':
                if 'robot_mode' not in payload:
                    raise ValueError('Missing field: robot_mode')
                robot_mode = int(payload['robot_mode'])
                success = self.server.node.set_robot_mode(robot_mode)  # type: ignore[attr-defined]
                mode_labels = {0: 'Manual', 1: 'Autonomous', 2: 'Measure'}
                message = f"Robot mode request sent (mode {robot_mode}: {mode_labels.get(robot_mode, 'Unknown')})."
                response_payload = {'success': success, 'robot_mode': robot_mode, 'message': message}
                if not success:
                    response_payload['error'] = 'SetRobotMode service reported failure.'
                self._send_json(200, response_payload)
                return
            else:
                if 'stop_mode' not in payload:
                    raise ValueError('Missing field: stop_mode')
                stop_mode = int(payload['stop_mode'])
                # ì¦‰ì‹œ HTTP ì‘ë‹µì„ ë³´ë‚´ê³ , ë³„ë„ ì“°ë ˆë“œì—ì„œ ë¹„ìƒì •ì§€ ì‹¤í–‰
                message = f"Emergency Stop request received (mode {stop_mode})"
                response_payload = {'success': True, 'stop_mode': stop_mode, 'message': message}
                self._send_json(200, response_payload)
                # HTTP ì‘ë‹µ í›„ ë³„ë„ ì“°ë ˆë“œì—ì„œ ë¹„ìƒì •ì§€ ë…¸ë“œë¡œ ì„œë¹„ìŠ¤ í˜¸ì¶œ
                threading.Thread(
                    target=lambda: self.server.estop_node.move_stop(stop_mode),  # type: ignore[attr-defined]
                    daemon=True
                ).start()
                return
        except (json.JSONDecodeError, ValueError) as exc:
            self._send_json(400, {'error': str(exc)})
        except Exception as exc:  # noqa: BLE001
            self._send_json(500, {'error': str(exc)})

    def log_message(self, format: str, *args: Any) -> None:  # noqa: A003
        return

    def _send_html(self, body: str) -> None:
        encoded = body.encode('utf-8')
        self.send_response(200)
        self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.send_header('Content-Length', str(len(encoded)))
        self._set_cors_headers()
        self.end_headers()
        self.wfile.write(encoded)

    def _send_json(self, status: int, payload: Dict[str, Any]) -> None:
        encoded = json.dumps(payload).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(encoded)))
        self._set_cors_headers()
        self.end_headers()
        self.wfile.write(encoded)

    def _set_cors_headers(self) -> None:
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')


class MotionHTTPServer(HTTPServer):
    def __init__(self, server_address: Tuple[str, int], handler: Type[BaseHTTPRequestHandler],
                 node: MotionClient, estop_node: EmergencyStopClient):
        super().__init__(server_address, handler)
        self.node = node
        self.estop_node = estop_node


def main(args: List[str] | None = None) -> None:
    rclpy.init(args=args)

    # ì¼ë°˜ ëª¨ì…˜ ì œì–´ ë…¸ë“œ
    node = MotionClient()

    # ë¹„ìƒì •ì§€ ì „ìš© ë…ë¦½ ë…¸ë“œ (ë³„ë„ executorì—ì„œ ì‹¤í–‰)
    estop_node = EmergencyStopClient()
    executor = MultiThreadedExecutor()
    executor.add_node(estop_node)

    # ë¹„ìƒì •ì§€ ë…¸ë“œë¥¼ ë³„ë„ ì“°ë ˆë“œì—ì„œ ì‹¤í–‰
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    # HTTP ì„œë²„ ì‹œìž‘
    server = MotionHTTPServer(('0.0.0.0', 8000), MotionRequestHandler, node, estop_node)
    node.get_logger().info('Serving MoveWeb UI at http://localhost:8000')
    estop_node.get_logger().info('Emergency stop handler ready (independent thread)')

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down motion web UI.')
    finally:
        server.server_close()
        executor.shutdown()
        node.destroy_node()
        estop_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
