from dashboard import dashboard2

simulbot_js = """
var robot_x = 0;
var robot_y = 0;
var robot_theta = 0
var robot_max_wheel_speed = {max_wheel_speed}
var robot_width = {robot_width};

var last_time = Date.now();

source.addEventListener('simulbot', function(event) {{

    var delta = Date.now() - last_time;
    last_time += delta;

    var data = JSON.parse(event.data);
    var left_dist = data.left * robot_max_wheel_speed * delta / 1000;
    var right_dist = data.right * robot_max_wheel_speed * delta / 1000;
    var dist = (left_dist + right_dist) / 2;
    robot_x += 12 * dist * Math.cos(robot_theta);
    robot_y += 12 * dist * Math.sin(robot_theta);
    robot_theta += (right_dist - left_dist) / robot_width


    var element = $("#simulbot");
    var canvas = element.get(0).getContext("2d");
    var h = element.height();
    var w = element.width();
    var SCALE = 1;
    var r_w = robot_width * 12;
    var r_h = 60 - r_w;
    r_w *= SCALE;
    r_h *= SCALE;

    canvas.clearRect(0, 0, w, h);
    canvas.beginPath();

    canvas.save();
    canvas.setTransform(1, 0, 0, 1, 0, 0);
    canvas.strokeStyle = "#fg0";
    canvas.translate(w/2, h/2);
    canvas.rotate(robot_theta + Math.PI / 2);

    canvas.rect(-r_w/2, -r_h/2, r_w, r_h);
    canvas.moveTo(0, 0);
    canvas.lineTo(0, -r_h/2);
    canvas.stroke();
    canvas.restore();

    canvas.save();
    canvas.setTransform(1, 0, 0, 1, 0, 0);
    canvas.strokeStyle = "#000";
    canvas.translate(w/2 -robot_x, h/2 -robot_y);
    var count = 50;
    var step = 12;

    for (var i = 0; i < 2 * count + 1; i++) {{
        canvas.moveTo(-count * step, -count * step + i * step);
        canvas.lineTo(count * step, -count * step + i * step)
    }}
    for (var i = 0; i < 2 * count + 1; i++) {{
        canvas.moveTo(-count * step + i * step, -count * step);
        canvas.lineTo(-count * step + i * step, count * step)
    }}

    canvas.stroke();
    canvas.restore();


}}, false);
"""


def load(robot_width, top_speed):
    dashboard2.extension("Simulbot", "<canvas id='simulbot' width='400' height='400'></canvas>",
                         simulbot_js.format(robot_width=robot_width, max_wheel_speed=top_speed), "")


def update(right_power, left_power):
    dashboard2.send_message({"left": left_power, "right": right_power}, "simulbot")