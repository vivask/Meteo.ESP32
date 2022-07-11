let canChangeLocation=true;
let radsensIsActive = false;
gel('rad-hv').disabled = true;
gel('valve-state').disabled = true;

docReady(async function () {  
    startLoadDataInterval();
});

function loadData() {
    JsonGetMethod("/ui/home/esp32datetime", success_esp32datetime)
    JsonGetMethod("/ui/home/bme280", success_bme280)
    JsonGetMethod("/ui/home/mics6814", success_mics6814)
    JsonGetMethod("/ui/home/radsens", success_radsens)
    JsonGetMethod("/ui/home/ds18b20", success_ds18b20)
    JsonGetMethod("/ui/home/ze08ch2o", success_ze08)
    JsonGetMethod("/ui/home/settings", success_alarms)
}

function success_esp32datetime(json){
    const duration = getEsp32Duration(json['esp32_date_time_now']);
    if(duration < MAX_DATA_UPDATE_PERIOD_MS){
        gel('clock').innerHTML = json['esp32_date_time'];
    }else{
        gel('clock').innerHTML = '';
    }
}

function success_ds18b20(json) {
    gel('ds18b20_t').innerHTML = fixeFloat(json["tempr"],1);
    const duration = getEsp32Duration(json['date_time']);
    if(duration < MAX_DATA_UPDATE_PERIOD_MS){
        gel('ds18b20_t').style.color = HOT_DATA;
    }else{
        gel('ds18b20_t').style.color = COLD_DATA;
    }
}

function success_bme280(json) {
    gel('bme280_t').innerHTML = fixeFloat(json["tempr"],1);
    gel('bme280_p').innerHTML = fixeFloat(json["press"]/133, 1);
    gel('bme280_h').innerHTML = fixeFloat(json["hum"],1);
    const duration = getEsp32Duration(json['date_time']);
    if(duration < MAX_DATA_UPDATE_PERIOD_MS){
        gel('bme280_t').style.color = HOT_DATA;
        gel('bme280_p').style.color = HOT_DATA;
        gel('bme280_h').style.color = HOT_DATA;
    }else{
        gel('bme280_t').style.color = COLD_DATA;
        gel('bme280_p').style.color = COLD_DATA;
        gel('bme280_h').style.color = COLD_DATA;
    }
}

function success_mics6814(json) {
    gel('nh3').innerHTML = json["nh3"];
    gel('no2').innerHTML = json["no2"];
    gel('co').innerHTML = json["co"];
    const duration = getEsp32Duration(json['date_time']);
    if(duration < MAX_DATA_UPDATE_PERIOD_MS){
        gel('nh3').style.color = HOT_DATA;
        gel('no2').style.color = HOT_DATA;
        gel('co').style.color = HOT_DATA;
    }else{
        gel('nh3').style.color = COLD_DATA;
        gel('no2').style.color = COLD_DATA;
        gel('co').style.color = COLD_DATA;
    }
}

function success_radsens(json) {
    gel('rad_stat').innerHTML = fixeFloat(json["static"],1);
    gel('rad_dyn').innerHTML = fixeFloat(json["dynamic"],1);
    const duration = getEsp32Duration(json['date_time']);
    if(duration < MAX_DATA_UPDATE_PERIOD_MS){
        gel('rad_stat').style.color = HOT_DATA;
        gel('rad_dyn').style.color = HOT_DATA;
        radsensIsActive = true;
    }else{
        gel('rad_stat').style.color = COLD_DATA;
        gel('rad_dyn').style.color = COLD_DATA;
        radsensIsActive = false;
    }
    gel('rad-hv').disabled = !radsensIsActive;
}

function success_ze08(json) {
    gel('ch2o').innerHTML = json["ch2o"];    
    const duration = getEsp32Duration(json['date_time']);
    if(duration < MAX_DATA_UPDATE_PERIOD_MS){
        gel('ch2o').style.color = HOT_DATA;
    }else{
        gel('ch2o').style.color = COLD_DATA;
    }
}

function success_alarms(json) {

    settingsDateTime = new Date(json["date_time"]);

    gel('t-alarm').style.display = (json["min_bmx280_tempr_alarm"] == 0 && json["max_bmx280_tempr_alarm"] == 0) ? 'none' : 'block';
    gel('nh3-alarm').style.display = (json["max_6814_nh3_alarm"] == 0) ? 'none' : 'block';
    gel('co-alarm').style.display = (json["max_6814_co_alarm"] == 0) ? 'none' : 'block';
    gel('no2-alarm').style.display = (json["max_6814_no2_alarm"] == 0) ? 'none' : 'block';
    gel('rad-stat-alarm').style.display = (json["max_rad_stat_alarm"] == 0) ? 'none' : 'block';
    gel('rad-dyn-alarm').style.display = (json["max_rad_dyn_alarm"] == 0) ? 'none' : 'block';
    gel('ds18b20-alarm').style.display = (json["min_ds18b20_alarm"] == 0 && json["max_ds18b20_alarm"] == 0) ? 'none' : 'block';
    gel('ch2o-alarm').style.display = (json["max_ch2o_alarm"] == 0) ? 'none' : 'block';

    gel('rad-hv').value = (json["radsens_hv_state"] == 0) ? 'Выключен' : 'Включен';
    if(radsensIsActive) gel('rad-hv').disabled = (json["radsens_hv_mode"] != 0);
    gel('valve-state').value = (json["valve_state"] == 0) ? 'Закрыт' : 'Открыт';

}

function td_in(id_name, class_name){
    gel(id_name).setAttribute('class', class_name);
}

function td_out(id_name, class_name){
    td_in(id_name, class_name);
}

function selectLocation(location){
    if(canChangeLocation){
        window.location=location;   
    }
}

function success_check(){
    canChangeLocation=true;
}

function onBme280TemperatureCheck(){
    canChangeLocation=false;
    JsonPutMethod("/ui/home/bme280", null, success_check);
}

async function onDs18b20Check(){ 
    canChangeLocation=false;
    JsonPutMethod("/ui/home/ds18b20", null, success_check);
}

async function onZe08Check(){
    canChangeLocation=false;
    JsonPutMethod("/ui/home/ze08ch2o", null, success_check);
}

function onRadStatCheck(){
    canChangeLocation=false;
    JsonPutMethod("/ui/home/radstat", null, success_check);
}

function onRadDynCheck(){
    canChangeLocation=false;
    JsonPutMethod("/ui/home/raddyn", null, success_check);
}

function onRadHV(el){
    canChangeLocation=false;
    let state = (el.value == 'Включен') ? 1 : 0;
    let answer, url;
    if(state == 1){
        answer = window.confirm("Выключить высоковольтный генератор?");
        url = "/ui/home/radhvset/0";
    }
    else{
        answer = window.confirm("Включить высоковольтный генератор?");
        url = "/ui/home/radhvset/1";
    }
    if(!answer){
        return;
    }

    gel('rad-hv').disabled = true;
    JsonPutMethod(url, null, success_check);
}
