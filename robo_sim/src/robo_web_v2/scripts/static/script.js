document.addEventListener("DOMContentLoaded", function() {
    const buttons = document.querySelectorAll(".diagonal-buttons button");
    const spinningButtons = document.querySelectorAll(".spinning-buttons button");
    // <p id="current_move">Current Move: Stop</p>
    const currentModeDiv = document.getElementById("current_move");
    
    /*
    forward > Forward
    backward > Backward
    right > Right Side
    left > Left Side
    stop > Stop
    spinlift > Spin Lift
    spinright > Spin Right
    digleftforward > Digonal Left Forward
    digrightforward > Digonal Right Forward
    digleftbackward > Diagonal Left Backward
    digrightbackward > Diagonal Back Right
    */

    
    buttons.forEach(button => {
        button.addEventListener("click", function() {
            const command = this.id;

            const commandMeanings = {
                forward: 'Forward',
                backward: 'Backward',
                right: 'Right Side',
                left: 'Left Side',
                stop: 'Stop',
                digleftforward: 'Diagonal Left Forward',
                digrightforward: 'Diagonal Right Forward',
                digleftbackward: 'Diagonal Left Backward',
                digrightbackward: 'Diagonal Right Backward'
            };

            const currentMoveMeaning = commandMeanings[command];

            // Set the HTML content with the current move and its meaning
            currentModeDiv.innerHTML = `${currentMoveMeaning}`;
            // print the command to the console
            console.log(command);
            sendCommand(command);
        });
    });
    spinningButtons.forEach(button => {
        button.addEventListener("click", function() {
            const command = this.id;
            const commandMeanings = {
                spinlift: 'Spin Left',
                spinright: 'Spin Right',
            };

            const currentMoveMeaning = commandMeanings[command];

            // Set the HTML content with the current move and its meaning
            currentModeDiv.innerHTML = `${currentMoveMeaning}`;
            // print the command to the console
            console.log(command);
            sendCommand(command);
        });
    });
});

function sendCommand(command) {
    // Send command to your server (Flask) endpoint
    fetch(`/send_command/${command}`, {
        method: "POST",
        headers: {
            "Content-Type": "application/json"
        },
        body: JSON.stringify({ command })
    })
    .then(response => {
        if (!response.ok) {
            throw new Error("Network response was not ok");
        }
        console.log("Command sent successfully");
    })
    .catch(error => {
        console.error("There was a problem with your fetch operation:", error);
    });
}
