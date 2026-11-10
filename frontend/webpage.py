import sys
from pathlib import Path
from flask import Flask, Response, render_template_string, jsonify
import logging

log = logging.getLogger("werkzeug")
log.setLevel(logging.ERROR)
PROJECT_ROOT = Path(__file__).resolve().parent.parent

PREDICTOR_DIR = (
    PROJECT_ROOT
    / "software"
    / "Firmware_Targets"
    / "Jetson"
    / "software_predictor"
)

sys.path.append(str(PREDICTOR_DIR))

from screencapGrabber import generate_frames, trackingStatus

app = Flask(__name__)


PAGE = """
<!DOCTYPE html>
<html>
<head>
    <title>Launcher Control</title>

    <style>
        body {
            margin: 0;
            height: 100vh;

            display: flex;
            align-items: center;
            justify-content: center;

            background: #111;
            font-family: Arial, sans-serif;
        }

        .fire-button {
    width: 320px;
    height: 320px;

    border-radius: 50%;
    border: 8px solid #c79a00;

    background: #ffd93d;
    color: black;

    font-size: 0;
    cursor: pointer;

    position: relative;
}

.fire-button::before {
    content: "●   ●";
    position: absolute;

    top: 75px;
    left: 0;
    width: 100%;

    font-size: 55px;
    text-align: center;
}

.fire-button::after {
    content: "⌣";
    position: absolute;

    top: 120px;
    left: 0;
    width: 100%;

    font-size: 125px;
    font-weight: bold;
    text-align: center;
}

.fire-button:active {
    transform: scale(0.96);
}
    </style>
</head>

<body>

    <form action="/fire" method="post">
        <button class="fire-button" type="submit">
            HI FRIEND!
        </button>
    </form>

</body>
</html>
"""


@app.route("/")
def index():
    return render_template_string(PAGE)


@app.route("/fire", methods=["GET", "POST"])
def fire():

    return """
    <html>
        <body style="
            background:#111;
            text-align:center;
            color:white;
        ">

            <h1>ZED Camera</h1>

            <img
                src="/video_feed"
                style="width:80%; max-width:900px;"
            >
        <div id="tracking">
    Waiting for ball...
</div>

<script>
    setInterval(async () => {
        const response = await fetch("/status");
        const data = await response.json();

        let text = "";

        if (data.current) {
            text +=
                `Ball: X=${data.current.x.toFixed(2)} ` +
                `Y=${data.current.y.toFixed(2)} ` +
                `Z=${data.current.z.toFixed(2)}<br>`;
        }

        if (data.final) {
            text +=
                `<strong>FINAL: ` +
                `X=${data.final.x.toFixed(2)} ` +
                `Y=${data.final.y.toFixed(2)} ` +
                `Z=${data.final.z.toFixed(2)}</strong>`;
        }

        document.getElementById("tracking").innerHTML =
            text || "Waiting for ball...";
    }, 500);
</script>
            


        </body>
    </html>
    """


@app.route("/video_feed")
def video_feed():

    return Response(
        generate_frames(),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )

@app.route("/status")
def status():
    return jsonify(trackingStatus)




if __name__ == "__main__":

    app.run(
        host="0.0.0.0",
        port=5001
    )