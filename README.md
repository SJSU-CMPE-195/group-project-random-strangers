# Project Title

> Workout form detector using Motion Capture cameras

## Team

| Name | GitHub | Email |
|------|--------|-------|
| Name 1 | [@ZoyaS](https://github.com/ZoyaS) | zoya.shamak@sjsu.edu |
| Name 2 | [@github210878](https://github.com/github210878) | isa.pudiyapura@sjsu.edu |
| Name 3 | [@eiechuang](https://github.com/eiechuang) | eric.huang01@sjsu.edu |
| Name 4 | [@brendanofawesome](https://github.com/Brendanofawesome) | brendan.parvin@sjsu.edu |

**Advisor:** [Prof. Wencen Wu]

---

## Problem Statement

Working out can be challenging for people who are unsure of their form, or are generally unfamiliar with the gym. While
weight training brings numerous health benefits, intimidation remains a barrier of access for many who do not. 

## Proof of Concept Scope

This POC is intended to demonstrate that we have a working flow. While a live demonstration is not possible yet, since we continue to configure the cameras, the project skeleton has been fleshed out. Cameras capture data, send it to a backend model, which then updates a website for the ease of use to the user. 

## Solution

Cameras that detect a workout being performed, and analyzes error between live process and ideal form through a trained model. 

### Key Features

- Motion Capture Cameras
- Internal workout model
- Live Feedback Website

---

## Screenshots

| Feature | Screenshot |
|---------|------------|
| [Online Server] | ![Screenshot](notes/ec2webserver.png) |
| [Feature 2] | ![Screenshot](docs/screenshots/feature2.png) |

---

## Tech Stack

| Category | Technology |
|----------|------------|
| Frontend | Next.JS, hosted online with Ec2 instance |
| Backend | Python Flask |
| Deployment | Kinect Studio/Optitrack Motive|

---

## Getting Started

This program is intended to run online, with no download for the user. Simply stand in front of the cameras and begin moving!


### Prerequisites

To run the project ~
- Next.JS v.10.0+
- Ultralytics YOLO v26
- 

### Installation

```bash
# Clone the repository
git clone https://github.com/SJSU-CMPE-195/group-project-random-strangers.git

# Install dependencies
(need a full package manager list)

# Set up environment variables
cp .env.example .env
# Edit .env with your values

# Run database migrations (if applicable)
[migration command]
```

### Running Locally

```bash
# Development mode
npm run build
npm run start

# The app will be available at http://localhost:3000
```

### Running Tests

```bash
[test command]
```

---
## Technical Stack

- Python, C++, HTML, JS, CSS, .TSX
= Next, Ultralytics, numpy, tensorflow
- Motion Capture Cameras, live Display 

## API Reference

- Need to be updated once website is live, probably next semester

<details>
<summary>Click to expand API endpoints</summary>

| Method | Endpoint | Description |
|--------|----------|-------------|
| GET | `/api/resource` | Get all resources |
| GET | `/api/resource/:id` | Get resource by ID |
| POST | `/api/resource` | Create new resource |
| PUT | `/api/resource/:id` | Update resource |
| DELETE | `/api/resource/:id` | Delete resource |

</details>

---

## Project Structure

```
.
├── [folder]/           # Description
├── src/                # Source code files
├── tests/              # Test files
├── docs/               # Documentation files
└── README.md
```

---

## What's Next (195B)

Change cameras from Temporary xbox kinects to cooler Optitrack motion cameras
decrease error percentage for weirdly shaped people
Live feedback, like workout coaching
Play around with motion capture technologies


## Acknowledgments

- [Resource/Library/Person]
- [Resource/Library/Person]

---

## License

This project is licensed under the <FILL IN> License - see the [LICENSE](LICENSE) file for details.

---

*CMPE 195A/B - Senior Design Project | San Jose State University | Spring 2026*
