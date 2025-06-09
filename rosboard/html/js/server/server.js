const express = require('express');
const { spawn } = require('child_process');
const fs = require('fs').promises;
const path = require('path');
const cors = require('cors');
const app = express();
const port = 3000;

// Global process variables for robot bringup, mapping, localization, and navigation
let launchProcess = null; // For robot bringup
let mappingProcess = null; // For mapping
let localizationOnlyProcess = null; // For localization only (load-map)
let navigationProcess = null; // For navigation (start-navigation)

// Middleware to parse JSON requests and enable CORS
app.use(express.json());
app.use(cors());

// Function to execute ROS2 commands with spawn
function runRos2Command(command, args) {
    return new Promise((resolve, reject) => {
        const proc = spawn('bash', ['-c', `source /opt/ros/humble/setup.bash && source ~/autofork_ws/install/setup.bash && ${command} ${args.map(arg => `'${arg.replace(/'/g, "'\\''")}'`).join(' ')}`]);
        let stdoutData = '';
        let stderrData = '';

        proc.stdout.on('data', (data) => {
            stdoutData += data.toString();
            console.log(`Stdout: ${data}`);
        });

        proc.stderr.on('data', (data) => {
            stderrData += data.toString();
            console.error(`Stderr: ${data}`);
        });

        proc.on('close', (code) => {
            if (code !== 0) {
                reject(new Error(`Command failed with code ${code}: ${stderrData}`));
            } else {
                resolve(stdoutData);
            }
        });

        proc.on('error', (error) => {
            reject(error);
        });
    });
}

// Function to get package share path
async function getPackageSharePath(packageName) {
    try {
        const output = await runRos2Command('ros2', ['pkg', 'prefix', '--share', packageName]);
        return output.trim();
    } catch (error) {
        console.error(`Failed to get share path for ${packageName}: ${error.message}`);
        return `/home/alip/autofork_ws/install/share/${packageName}`;
    }
}

// Check if a process is still running
function isProcessRunning(process) {
    return process && !process.killed && process.exitCode === null;
}

// Function to stop localization process
function stopLocalizationProcess() {
    return new Promise((resolve, reject) => {
        if (!localizationOnlyProcess || !isProcessRunning(localizationOnlyProcess)) {
            localizationOnlyProcess = null;
            resolve();
            return;
        }

        try {
            localizationOnlyProcess.kill('SIGINT');
            console.log(`Sent SIGINT to localization process with PID ${localizationOnlyProcess.pid} at ${new Date().toISOString()}`);

            localizationOnlyProcess.on('close', (code) => {
                console.log(`Localization process exited with code ${code} at ${new Date().toISOString()}`);
                localizationOnlyProcess = null;
                resolve();
            });

            localizationOnlyProcess.on('error', (error) => {
                console.error(`Error stopping localization process: ${error.message}`);
                localizationOnlyProcess = null;
                reject(error);
            });
        } catch (error) {
            console.error(`Error stopping localization process: ${error.message}`);
            localizationOnlyProcess = null;
            reject(error);
        }
    });
}

// Save map: Run save_map service call and organize files
app.post('/save-map', async (req, res) => {
    const { mapName } = req.body;
    if (!mapName || typeof mapName !== 'string') {
        console.error('Invalid mapName:', mapName);
        return res.status(400).json({ success: false, error: 'Invalid or missing mapName' });
    }

    const safeMapName = mapName.replace(/[^a-zA-Z0-9_-]/g, '_').toLowerCase();
    const sharePath = await getPackageSharePath('autofork_navigation');
    const installMapDir = path.join(sharePath, 'maps', safeMapName);
    const installFilename = path.join(installMapDir, safeMapName);
    const srcMapDir = `/home/alip/autofork_ws/src/autofork/autofork_navigation/maps/${safeMapName}`;
    const tempMapDir = `/tmp/rosboard_maps`;

    try {
        // Create temporary directory
        await fs.mkdir(tempMapDir, { recursive: true });
        console.log(`Created temporary directory: ${tempMapDir}`);

        // Save to installed directory
        await fs.mkdir(installMapDir, { recursive: true });
        console.log(`Created directory: ${installMapDir}`);
        const installCommand = 'ros2';
        const installArgs = [
            'service', 'call', '/slam_toolbox/save_map',
            'slam_toolbox/srv/SaveMap',
            `{name: {data: "${installFilename}"}}`
        ];
        console.log(`Executing install command: ${installCommand} ${installArgs.join(' ')}`);
        await runRos2Command(installCommand, installArgs);

        // Save to source directory
        await fs.mkdir(srcMapDir, { recursive: true });
        console.log(`Created directory: ${srcMapDir}`);
        const defaultCommand = 'ros2';
        const defaultArgs = [
            'service', 'call', '/slam_toolbox/save_map',
            'slam_toolbox/srv/SaveMap',
            `{name: {data: "${path.join(tempMapDir, safeMapName)}"}}`
        ];
        console.log(`Executing save_map command: ${defaultCommand} ${defaultArgs.join(' ')}`);
        await runRos2Command(defaultCommand, defaultArgs);

        // Move files from temporary to source directory
        const tempPgm = path.join(tempMapDir, `${safeMapName}.pgm`);
        const tempYaml = path.join(tempMapDir, `${safeMapName}.yaml`);
        const srcPgm = path.join(srcMapDir, `${safeMapName}.pgm`);
        const srcYaml = path.join(srcMapDir, `${safeMapName}.yaml`);

        try {
            await fs.access(tempPgm);
            await fs.access(tempYaml);
            await fs.rename(tempPgm, srcPgm);
            await fs.rename(tempYaml, srcYaml);
            console.log(`Moved ${safeMapName}.pgm and ${safeMapName}.yaml to ${srcMapDir}`);
        } catch (moveError) {
            console.error(`Failed to move files: ${moveError.message}`);
            throw new Error(`Failed to move map files to ${srcMapDir}: ${moveError.message}`);
        }

        res.json({ success: true, message: `Saved map '${safeMapName}' to ${installMapDir} and ${srcMapDir}` });
    } catch (error) {
        console.error(`Failed to save map '${safeMapName}': ${error.message}`);
        res.status(500).json({ success: false, error: error.message });
    }
});

// Turn on robot: Launch the robot bringup
app.post('/turn-on', (req, res) => {
    if (launchProcess && isProcessRunning(launchProcess)) {
        return res.status(400).json({ success: false, error: 'Robot launch process already running' });
    }

    const command = 'bash';
    const args = ['-c', 'source /opt/ros/humble/setup.bash && source ~/autofork_ws/install/setup.bash && ros2 launch autofork_launch autofork_gazebo_control.launch.py'];

    try {
        launchProcess = spawn(command, args, { stdio: ['ignore', 'pipe', 'pipe'] });
        console.log(`Started robot launch process with PID ${launchProcess.pid} at ${new Date().toISOString()}`);

        let stdoutData = '';
        let stderrData = '';
        launchProcess.stdout.on('data', (data) => {
            stdoutData += data.toString();
            console.log(`Robot launch stdout: ${data}`);
        });
        launchProcess.stderr.on('data', (data) => {
            stderrData += data.toString();
            console.error(`Robot launch stderr: ${data}`);
        });

        launchProcess.on('close', (code) => {
            console.log(`Robot launch process exited with code ${code} at ${new Date().toISOString()}`);
            launchProcess = null;
            if (code !== 0) {
                console.error(`Robot launch process failed: ${stderrData}`);
            }
        });

        launchProcess.on('error', (error) => {
            console.error(`Robot launch process error: ${error.message}`);
            launchProcess = null;
        });

        res.json({ success: true, message: 'Robot launch process started' });
    } catch (error) {
        console.error(`Error starting robot launch process: ${error.message}`);
        res.status(500).json({ success: false, error: error.message });
    }
});

// Turn off robot: Stop the robot bringup
app.post('/turn-off', (req, res) => {
    if (!launchProcess || !isProcessRunning(launchProcess)) {
        launchProcess = null;
        return res.status(400).json({ success: false, error: 'No robot launch process running' });
    }

    try {
        launchProcess.kill('SIGINT');
        console.log(`Sent SIGINT to robot launch process with PID ${launchProcess.pid} at ${new Date().toISOString()}`);
        res.json({ success: true, message: 'Robot launch process stopped' });
    } catch (error) {
        console.error(`Error stopping robot launch process: ${error.message}`);
        res.status(500).json({ success: false, error: error.message });
    }
});

// Start mapping: Launch the mapping process
app.post('/start-mapping', (req, res) => {
    if (mappingProcess && isProcessRunning(mappingProcess)) {
        return res.status(400).json({ success: false, error: 'Mapping process already running' });
    }

    const command = 'bash';
    const args = ['-c', 'source /opt/ros/humble/setup.bash && source ~/autofork_ws/install/setup.bash && ros2 launch autofork_launch online_async_mapper_launch.py slam_params_file:=$(ros2 pkg prefix autofork_navigation)/share/autofork_navigation/config/slam_config/mapper_params_online_async.yaml use_sim_time:=true'];

    try {
        mappingProcess = spawn(command, args, { stdio: ['ignore', 'pipe', 'pipe'] });
        console.log(`Started mapping process with PID ${mappingProcess.pid} at ${new Date().toISOString()}`);

        let stdoutData = '';
        let stderrData = '';
        mappingProcess.stdout.on('data', (data) => {
            stdoutData += data.toString();
            console.log(`Mapping stdout: ${data}`);
        });
        mappingProcess.stderr.on('data', (data) => {
            stderrData += data.toString();
            console.error(`Mapping stderr: ${data}`);
        });

        mappingProcess.on('close', (code) => {
            console.log(`Mapping process exited with code ${code} at ${new Date().toISOString()}`);
            mappingProcess = null;
            if (code !== 0) {
                console.error(`Mapping process failed: ${stderrData}`);
            }
        });

        mappingProcess.on('error', (error) => {
            console.error(`Mapping process error: ${error.message}`);
            mappingProcess = null;
        });

        res.json({ success: true, message: 'Mapping process started' });
    } catch (error) {
        console.error(`Error starting mapping process: ${error.message}`);
        res.status(500).json({ success: false, error: error.message });
    }
});

// Stop mapping: Stop the mapping process
app.post('/stop-mapping', (req, res) => {
    if (!mappingProcess || !isProcessRunning(mappingProcess)) {
        mappingProcess = null;
        return res.status(400).json({ success: false, error: 'No mapping process running' });
    }

    try {
        mappingProcess.kill('SIGINT');
        console.log(`Sent SIGINT to mapping process with PID ${mappingProcess.pid} at ${new Date().toISOString()}`);
        res.json({ success: true, message: 'Mapping process stopped' });
    } catch (error) {
        console.error(`Error stopping mapping process: ${error.message}`);
        res.status(500).json({ success: false, error: error.message });
    }
});

// Full lift: Publish to /lift_control
app.post('/full-lift', async (req, res) => {
    try {
        const command = 'ros2';
        const args = [
            'topic', 'pub', '--once', '/lift_control',
            'std_msgs/Bool',
            '{data: true}'
        ];
        console.log(`Executing lift command: ${command} ${args.join(' ')}`);
        await runRos2Command(command, args);
        res.json({ success: true, message: 'Full lift command sent' });
    } catch (error) {
        console.error(`Error sending full lift command: ${error.message}`);
        res.status(500).json({ success: false, error: error.message });
    }
});

// Full lower: Publish to /lift_control
app.post('/full-lower', async (req, res) => {
    try {
        const command = 'ros2';
        const args = [
            'topic', 'pub', '--once', '/lift_control',
            'std_msgs/Bool',
            '{data: false}'
        ];
        console.log(`Executing lower command: ${command} ${args.join(' ')}`);
        await runRos2Command(command, args);
        res.json({ success: true, message: 'Full lower command sent' });
    } catch (error) {
        console.error(`Error sending full lower command: ${error.message}`);
        res.status(500).json({ success: false, error: error.message });
    }
});

// Start navigation: Launch navigation process
app.post('/start-navigation', async (req, res) => {
    if (navigationProcess && isProcessRunning(navigationProcess)) {
        return res.status(400).json({ success: false, error: 'Navigation process already running' });
    }

    const command = 'bash';
    const args = ['-c', 'source /opt/ros/humble/setup.bash && source ~/autofork_ws/install/setup.bash && ros2 launch nav2_bringup navigation_launch.py'];

    try {
        navigationProcess = spawn(command, args, { stdio: ['ignore', 'pipe', 'pipe'] });
        console.log(`Started navigation process with PID ${navigationProcess.pid} at ${new Date().toISOString()}`);

        let stdoutData = '';
        let stderrData = '';
        navigationProcess.stdout.on('data', (data) => {
            stdoutData += data.toString();
            console.log(`Navigation stdout: ${data}`);
        });
        navigationProcess.stderr.on('data', (data) => {
            stderrData += data.toString();
            console.error(`Navigation stderr: ${data}`);
        });

        navigationProcess.on('close', (code) => {
            console.log(`Navigation process exited with code ${code} at ${new Date().toISOString()}`);
            navigationProcess = null;
            if (code !== 0) {
                console.error(`Navigation process failed: ${stderrData}`);
            }
        });

        navigationProcess.on('error', (error) => {
            console.error(`Navigation process error: ${error.message}`);
            navigationProcess = null;
        });

        res.json({ success: true, message: 'Navigation process started' });
    } catch (error) {
        console.error(`Error starting navigation process: ${error.message}`);
        res.status(500).json({ success: false, error: error.message });
    }
});

// Stop navigation: Stop navigation process
app.post('/stop-navigation', (req, res) => {
    if (!navigationProcess || !isProcessRunning(navigationProcess)) {
        navigationProcess = null;
        return res.status(400).json({ success: false, error: 'No navigation process running' });
    }

    try {
        navigationProcess.kill('SIGINT');
        console.log(`Sent SIGINT to navigation process with PID ${navigationProcess.pid} at ${new Date().toISOString()}`);
        res.json({ success: true, message: 'Navigation process stopped' });
    } catch (error) {
        console.error(`Error stopping navigation process: ${error.message}`);
        res.status(500).json({ success: false, error: error.message });
    }
});

// Load map for navigation: Launch localization with specified map
app.post('/load-map', async (req, res) => {
    const { mapName } = req.body;
    if (!mapName || typeof mapName !== 'string') {
        console.error('Invalid mapName:', mapName);
        return res.status(400).json({ success: false, error: 'Invalid or missing mapName' });
    }

    const safeMapName = mapName.replace(/[^a-zA-Z0-9_-]/g, '_').toLowerCase();
    const mapDir = `/home/alip/autofork_ws/src/autofork/autofork_navigation/maps/${safeMapName}`;
    const mapFile = path.join(mapDir, `${safeMapName}.yaml`);

    try {
        await fs.access(mapFile);
    } catch (error) {
        console.error(`Map file ${mapFile} does not exist: ${error.message}`);
        return res.status(400).json({ success: false, error: `Map file ${safeMapName}.yaml does not exist in ${mapDir}` });
    }

    // Stop existing localization process if running
    try {
        await stopLocalizationProcess();
        console.log(`Previous localization process stopped (if any) at ${new Date().toISOString()}`);
    } catch (error) {
        console.error(`Failed to stop previous localization process: ${error.message}`);
        return res.status(500).json({ success: false, error: `Failed to stop previous localization process: ${error.message}` });
    }

    const command = 'bash';
    const args = ['-c', `source /opt/ros/humble/setup.bash && source ~/autofork_ws/install/setup.bash && ros2 launch autofork_launch localization_launch.py map:=${mapFile} use_sim_time:=true`];

    try {
        localizationOnlyProcess = spawn(command, args, { stdio: ['ignore', 'pipe', 'pipe'] });
        console.log(`Started localization process with PID ${localizationOnlyProcess.pid} for map '${safeMapName}' at ${new Date().toISOString()}`);

        let stdoutData = '';
        let stderrData = '';
        localizationOnlyProcess.stdout.on('data', (data) => {
            stdoutData += data.toString();
            console.log(`Localization stdout: ${data}`);
        });
        localizationOnlyProcess.stderr.on('data', (data) => {
            stderrData += data.toString();
            console.error(`Localization stderr: ${data}`);
        });

        localizationOnlyProcess.on('close', (code) => {
            console.log(`Localization process exited with code ${code} at ${new Date().toISOString()}`);
            localizationOnlyProcess = null;
            if (code !== 0) {
                console.error(`Localization process failed: ${stderrData}`);
            }
        });

        localizationOnlyProcess.on('error', (error) => {
            console.error(`Localization process error: ${error.message}`);
            localizationOnlyProcess = null;
        });

        res.json({ success: true, message: `Localization process started with map '${safeMapName}'` });
    } catch (error) {
        console.error(`Error starting localization process for map '${safeMapName}': ${error.message}`);
        res.status(500).json({ success: false, error: error.message });
    }
});

// Start server
app.listen(port, () => {
    console.log(`Server running at http://localhost:${port}`);
});