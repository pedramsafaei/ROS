import React from "react";
import ReactDOM from "react-dom";
import Particles from 'react-particles-js';
import * as cocoSsd from "@tensorflow-models/coco-ssd";
import Media from 'react-media';
import "@tensorflow/tfjs";
import "./index.css";

class App extends React.Component {
  state = { width: window.innerWidth, height: window.innerHeight }


  componentDidMount() {
    if(this.state.width <= 480 ){
      const video = document.getElementById("video");
      const webCamPromise = navigator.mediaDevices
        .getUserMedia({
          audio: false,
          video: {
            facingMode: "user",
            width: this.state.width,
            height: this.state.height
          }
        })
        .then(stream => {
          video.srcObject = stream;
          return new Promise((resolve, reject) => {
            video.onloadedmetadata = () => {
              video.play();
              resolve();
            };
          });
        });
      const modelPromise = cocoSsd.load();
      Promise.all([modelPromise, webCamPromise]).then(values => {
        this.detectFrame(video, values[0]);
      });
      console.log("SMALL DEVICE")
      console.log(this.state.width)
      console.log(this.state.height)
    }else{
      const video = document.getElementById("video");
      const webCamPromise = navigator.mediaDevices
        .getUserMedia({
          audio: false,
          video: {
            facingMode: "user",
            width: this.state.width,
            height: this.state.height / 1.1
          }
        })
        .then(stream => {
          video.srcObject = stream;
          return new Promise((resolve, reject) => {
            video.onloadedmetadata = () => {
              video.play();
              resolve();
            };
          });
        });
      const modelPromise = cocoSsd.load();
      Promise.all([modelPromise, webCamPromise]).then(values => {
        this.detectFrame(video, values[0]);
      });
      console.log("LARGE DEVICE")
      console.log(this.state.width)
      console.log(this.state.height)
    }

  }

  detectFrame = (video, model) => {
    model.detect(video).then(predictions => {
      this.renderPredictions(predictions);
      requestAnimationFrame(() => {
        this.detectFrame(video, model);
      });
    });
  };

  renderPredictions = predictions => {
    const c = document.getElementById("canvas");
    const ctx = c.getContext("2d");
    ctx.clearRect(0, 0, ctx.canvas.width, ctx.canvas.height);
    // Font options.
    const font = "25px Noto Sans";
    ctx.font = font;
    ctx.textBaseline = "top";
    predictions.forEach(prediction => {
      const x = prediction.bbox[0];
      const y = prediction.bbox[1];
      const width = prediction.bbox[2];
      const height = prediction.bbox[3];
      // Draw the bounding box.
      ctx.strokeStyle = "#e74c3c";
      ctx.lineWidth = 4;
      ctx.strokeRect(x, y, width, height);
      // Draw the label background.
      ctx.fillStyle = "#e74c3c";
      const textWidth = ctx.measureText(prediction.class).width;
      const textHeight = parseInt(font, 20); // base 10
      ctx.fillRect(x, y, textWidth + 4, textHeight + 4);
    });

    predictions.forEach(prediction => {
      const x = prediction.bbox[0];
      const y = prediction.bbox[1];
      // Draw the text last to ensure it's on top.
      ctx.fillStyle = "#FFFFFF";
      ctx.fillText(prediction.class, x, y);
    });
  };

  render() {
    return (
      <div className="container">
        <Media query={{ maxWidth: 480 }}>
          {matches => matches ? (
            <div>
              <Particles
                params={{
                  "particles": {
                    "number": {
                      "value": 160,
                      "density": {
                        "enable": false
                      }
                    },
                    "color": {
                      "value": ["#f1c40f", "#B8E986", "#50E3C2", "#FFD300", "#E86363", "#e74c3c"]
                    },
                    "shape": {
                      "type": "circle",
                      "stroke": {
                        "width": 0,
                        "color": "#e74c3c"
                      }
                    },
                    "size": {
                      "value": 3,
                      "random": true,
                      "anim": {
                        "speed": 4,
                        "size_min": 0.3
                      }
                    },
                    "line_linked": {
                      "enable": false
                    },
                    "move": {
                      "random": true,
                      "speed": 1,
                      "direction": "top",
                      "out_mode": "out"
                    }
                  },
                  "interactivity": {
                    "events": {
                      "onhover": {
                        "enable": true,
                        "mode": "bubble"
                      },
                      "onclick": {
                        "enable": true,
                        "mode": "repulse"
                      }
                    },
                    "modes": {
                      "bubble": {
                        "distance": 250,
                        "duration": 2,
                        "size": 0,
                        "opacity": 0
                      },
                      "repulse": {
                        "distance": 400,
                        "duration": 4
                      }
                    }
                  }
                }} />
              <video id="video" className="higher" width={this.state.width} height={this.state.height} />
              <canvas id="canvas" className="higher" width={this.state.width} height={this.state.height} />
            </div>
          ) : (

              <div>
                <Particles
                  params={{
                    "particles": {
                      "number": {
                        "value": 160,
                        "density": {
                          "enable": false
                        }
                      },
                      "color": {
                        "value": ["#f1c40f", "#B8E986", "#50E3C2", "#FFD300", "#E86363", "#e74c3c"]
                      },
                      "shape": {
                        "type": "circle",
                        "stroke": {
                          "width": 0,
                          "color": "#e74c3c"
                        }
                      },
                      "size": {
                        "value": 3,
                        "random": true,
                        "anim": {
                          "speed": 4,
                          "size_min": 0.3
                        }
                      },
                      "line_linked": {
                        "enable": false
                      },
                      "move": {
                        "random": true,
                        "speed": 1,
                        "direction": "top",
                        "out_mode": "out"
                      }
                    },
                    "interactivity": {
                      "events": {
                        "onhover": {
                          "enable": true,
                          "mode": "bubble"
                        },
                        "onclick": {
                          "enable": true,
                          "mode": "repulse"
                        }
                      },
                      "modes": {
                        "bubble": {
                          "distance": 250,
                          "duration": 2,
                          "size": 0,
                          "opacity": 0
                        },
                        "repulse": {
                          "distance": 400,
                          "duration": 4
                        }
                      }
                    }
                  }} />
                <video id="video" className="higher" width={this.state.width} height={this.state.height / 1.1} />
                <canvas id="canvas" className="higher" width={this.state.width} height={this.state.height / 1.1} />
              </div>
            )
          }
        </Media>
      </div>
    );
  }
}

const rootElement = document.getElementById("root");
ReactDOM.render(<App />, rootElement);
