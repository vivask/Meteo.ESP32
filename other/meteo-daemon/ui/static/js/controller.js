let settingsDateTime = new Date();
let currentPage=gel('pfirmware');
let currentUpgradeStatus=gel('upgrading');
let currentSetupStatus=gel('setuping');
let currentRestartStatus=gel('restarting');

const RESPONSE_MAX = 3000;
const RESPONSE_UPGRADE_MAX = 10000;
let response_count = 0;

docReady(async function () {  
    gel('fw-save').disabled = true;
    gel('fw-setup').disabled = true;
    gel('fw-restart').disabled = true;  
    startLoadDataInterval();
});

function loadData() {
    JsonGetMethod("/ui/home/esp32datetime", success_esp32datetime)
}

function success_esp32datetime(json){
    const recv_date = new Date(json['esp32_date_time_now']);
    const duration = Date.now() - recv_date;

    console.log("Duration: " + duration);
    if(duration < MAX_DATA_UPDATE_PERIOD_MS){
        gel('clock').innerHTML = json['esp32_date_time'];
        checkActiveESP32(false);
    }else{
        gel('clock').innerHTML = '';
        checkActiveESP32(true);
    }
}

function checkActiveESP32(disabled){
    gel('fw-save').disabled = (gel('fw_file').value.length == 0 || disabled);
    gel('fw-setup').disabled = disabled;
    gel('fw-restart').disabled = disabled;    
}

function select_page(page_id){
    selectPage(page_id, currentPage);
}

async function onSaveFw(){
    let answer = window.confirm("Выполнить прошивку контроллера?");
    if(answer){
        let files = gel('fw_file').files;
        let data = new FormData();

        for (let i = 0; i < files.length; i++) {
            let file = files[i]    
            data.append('uploadfile', file)
        }

        await fetch('/upload', {
            method: 'POST', 
            body: data,
        }).then(success_firmware_upload(files.item(0).name))
        .catch(error => failure(error)); 
    }
}

function success_firmware_upload(fname){
    response_count = 0;

    showWaitWin('popup-wait');

    url = "/verify/controller/upgrade/"+fname;
    JsonPutMethod(url, null, firmware_upgrade_started);
}

function firmware_upgrade_started(json){
    JsonGetMethod("/verify/controller/upgrade/status", success_upgrade_status);
}

function success_upgrade_status(json){
    let response = json["upgrade_status"];
    if(response == '0'){
        if(response_count < RESPONSE_UPGRADE_MAX){
            response_count=response_count+1;
            firmware_upgrade_started(null);    
        }else{
            select_upgrade_status('upgrade-timeout');
            JsonPutMethod("/verify/controller/upgrade/terminate", null, success);
            hideWaitWin('popup-wait');
        }
    }else{
        if(response == '1'){
            select_upgrade_status('upgrade-success');
            hideWaitWin('popup-wait');
        }else{
            select_upgrade_status('upgrade-fail');
            hideWaitWin('popup-wait');
        }
    }
}

function select_upgrade_status(upgrade_status_id){
    selectStatus(upgrade_status_id, currentUpgradeStatus);
}

function onSetupMode(){
    let answer = window.confirm("Переключить контроллер в режим первичной настройки?");
    if(answer){
        JsonPutMethod("/verify/controller/ap", null, success_setup_mode);
    }
}

function success_setup_mode(json){
    response_count = 0;
    showWaitWin('popup-wait');
    select_setup_status('setuping');

    JsonGetMethod("/verify/controller/ap/status", success_setup_status);       
}

function success_setup_status(json){
    let response = json["setup_status"];
    if(response == '0'){
        if(response_count < RESPONSE_MAX){
            response_count=response_count+1;
        }else{
            select_setup_status('setup-mode-timeout');
        }
    }else{
        window.location=location;
    }
    hideWaitWin('popup-wait');
}

function select_setup_status(setup_status_id){
    selectStatus(setup_status_id, currentSetupStatus);
}

function onRestart(){
    let answer = window.confirm("Перезапустить контроллер?");
    if(answer){
        response_count = 0;
        JsonPutMethod("/verify/controller/reboot", null, success_restart);
    }
}

async function success_restart(json){
    response_count = 0;
    restartingESP32 = true;
    showWaitWin('popup-wait');
    select_restart_status('restarting');
    await delay(500);
    JsonGetMethod("/verify/controller/reboot/status", success_restart_status);
}

function success_restart_status(json){
    let response = json["rebooted"];
    if(response == '1'){
        window.location=location; 
    }else{
        if(response_count < RESPONSE_MAX){
            response_count=response_count+1;
        }else{
            select_restart_status('restart-timeout');
        }
    }
    hideWaitWin('popup-wait');
    restartingESP32 = false;
}

function select_restart_status(restart_status_id){
    selectStatus(restart_status_id, currentRestartStatus);
}


function onCancel(){
    window.location=location;    
}
