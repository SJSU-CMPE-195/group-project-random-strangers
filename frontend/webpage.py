from flask import Flask, render_template_string
import subprocess
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent

SCREENCAP_PATH = (
    BASE_DIR
    / "software_predictor"
    / "screencapGrabber.py"
)



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

    print("Starting ZED camera...")

    subprocess.Popen([
        "python3",
        str(SCREENCAP_PATH)
    ])

    return render_template_string(PAGE)


if __name__ == "__main__":
    app.run(
        host="0.0.0.0",
        port=5001
    )