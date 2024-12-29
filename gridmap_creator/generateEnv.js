// generateEnv.js
// home directoryのパスを.envファイルに保存するためのスクリプト。
const fs = require('fs');
const os = require('os');

const homeDir = os.homedir();

const envContent = `REACT_APP_HOME_DIR=${homeDir}\n`;

fs.writeFileSync('.env', envContent, 'utf8');

console.log('.env file has been generated with the following content:');
console.log(envContent);
