#ifndef SERVER_H
#define SERVER_H

#include <stdint.h>
#include <atomic>
#include <vector> 
#include <ESP32Servo.h>  

/*
extern const std::vector<std::vector<double>> square_path;
extern const std::vector<std::vector<double>> spin_path;
extern const std::vector<std::vector<double>> pitch_sweep_path;
extern const std::vector<std::vector<double>>* current_path;
*/

extern std::atomic<uint32_t> crStopAt;
extern bool manual_mode;  

const char CONTROL_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
  <head>
    <meta charset="utf-8">
    <title>CTRL ARM DEL</title>
    <meta name="viewport" content="width=device-width,initial-scale=1">
    <style>
      body {
        font-family:sans-serif;margin:2rem;max-width:480px
      }
      
      h1 {
        font-size:1.4rem;margin-bottom:1rem
      }
      
      .slider-block {
        margin:1rem 0
      }
      
      label {
        display:block;font-weight:bold;margin-bottom:0.3rem
      }
      
      input[type=range] {
        width:100%%;
      }        
    </style>
  </head>
  <body>
  <h1>CTRL the ARM</h1>
  
  <div id="sliders"></div>
    
    <script>
      const SERVO_COUNT = %SERVO_COUNT%;      
      const socket = new WebSocket(`ws://${location.host}/ws`);
      
      socket.addEventListener('open', () => console.log('WS connected'));
      
      function sendAngle(id, deg){
        if(socket.readyState===1) socket.send(JSON.stringify({id,deg}));
      }
      
      function buildUI(){
        const wrap=document.getElementById('sliders');
        for(let i=0; i<SERVO_COUNT; i++) {
          const block = document.createElement('div');
          block.className = 'slider-block';
          block.innerHTML = `
            <label for="s${i}">Servo ${i}</label>
            <input id="s${i}" type="range" min="0" max="180" value="90">
          `;
          block.querySelector('input').addEventListener('input', e=>{
            sendAngle(i, parseInt(e.target.value));
          });
          wrap.appendChild(block);
        }
      }
      
      buildUI();
    </script>
  </body>
</html>
)rawliteral";

void init_server();
void server_setup();
void server_loop();

#endif