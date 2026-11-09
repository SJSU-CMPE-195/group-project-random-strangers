import sys
from pathlib import Path
from flask import Flask, Response, render_template_string


from screencapGrabber import generate_frames

PROJECT_ROOT = Path(__file__).resolve().parent.parent

PREDICTOR_DIR = (
    PROJECT_ROOT
    / "software"
    / "Firmware_Targets"
    / "Jetson"
    / "software_predictor"
)

sys.path.append(str(PREDICTOR_DIR))


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
            border: 12px solid #7a0000;

            background: red;
            color: white;

            font-size: 64px;
            font-weight: bold;

            cursor: pointer;
        }

        .fire-button:active {
            transform: scale(0.96);
            background: #cc0000;
        }
    </style>
</head>

<body>

    <form action="/fire" method="post">
        <button class="fire-button" type="submit">
            FIRE
        </button>
    </form>

</body>
</html>
"""


@app.route("/")
def index():
    return render_template_string(PAGE)


@app.route("/fire", methods=["POST"])
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

        </body>
    </html>
    """


@app.route("/video_feed")
def video_feed():

    return Response(
        generate_frames(),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )

if __name__ == "__main__":
    app.run(
        host="0.0.0.0",
        port=5001
    )