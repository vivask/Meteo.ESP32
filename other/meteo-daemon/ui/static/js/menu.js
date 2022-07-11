
docReady(async function () {  
    startLoadDataInterval();
});

function loadData() {
    JsonGetMethod("/ui/home/esp32datetime", success_esp32datetime)
}

function success_esp32datetime(json){
    const recv_date = new Date(json['esp32_date_time_now']);
    const now = Date.now();
    if(now-recv_date < MAX_DATA_UPDATE_PERIOD_MS){
        gel('clock').innerHTML = json['esp32_date_time'];
    }else{
        gel('clock').innerHTML = '';
    }
}