let currentPage=gel('psettings');
let currentSettingsStatus=gel('settings-success');

docReady(async function () {  
    JsonGetMethod("/verify/settings/get", success_settings_data);
    //checkSettingsFill();
});

function success_settings_data(json){
    gel('bme280_min_t_text').value = json['min_bmx280_tempr'];
    gel('bme280_max_t_text').value = json['max_bmx280_tempr'];
    gel('nh3_text').value = json['max_6814_nh3'];
    gel('co_text').value = json['max_6814_co'];
    gel('no2_text').value = json['max_6814_no2'];
    gel('rad_stat_text').value = json['max_rad_stat'];
    gel('rad_dyn_text').value = json['max_rad_dyn'];
    gel('rad_sensitivity_text').value = json['radsens_sensitivity'];
    gel('ds18b20_min_text').value = json['min_ds18b20'];
    gel('ds18b20_max_text').value = json['max_ds18b20'];
    gel('ze08_text').value = json['max_ch2o'];
    gel('valve_min_text').value = json['min_temp'];
    gel('valve_max_text').value = json['max_temp'];
    if(json["valve_disable"] == 0){
        gel('select-valve-disabled').value = 'no';
    }else{
        gel('select-valve-disabled').value = 'yes';
    }
}

function onSaveSets(){
    let cb = gel('select-valve-disabled');
    let data = {
        max_6814_nh3: parseFloat(gel('nh3_text').value),
        max_6814_co: parseFloat(gel('co_text').value),
        max_6814_no2: parseFloat(gel('no2_text').value),
        max_rad_stat: parseFloat(gel('rad_stat_text').value),
        max_rad_dyn: parseFloat(gel('rad_dyn_text').value),
        min_ds18b20: parseFloat(gel('ds18b20_min_text').value),
        max_ds18b20: parseFloat(gel('ds18b20_max_text').value),
        max_ch2o: parseInt(gel('ze08_text').value),
        min_temp: parseFloat(gel('valve_min_text').value),
        max_temp: parseFloat(gel('valve_max_text').value),
        valve_disable: (cb.options[cb.selectedIndex].value == 'yes') ? true : false,
        min_bmx280_tempr : parseFloat(gel('bme280_min_t_text').value),
        max_bmx280_tempr : parseFloat(gel('bme280_max_t_text').value),
        radsens_sensitivity: parseInt(gel('rad_sensitivity_text').value)
    };
    JsonPostMethod("/verify/settings/set", data, success_reload);
}

function validate(){
    if(isValidFloat('nh3_text') && isValidFloat('co_text') && isValidFloat('no2_text') &&
       isValidFloat('rad_stat_text') && isValidFloat('rad_dyn_text') &&
       isValidFloat('ze08_text') && isValidFloat('valve_min_text') && isValidFloat('valve_max_text') && 
       isValidFloat('bme280_min_t_text') && isValidFloat('bme280_max_t_text') && 
       isValidFloat('ds18b20_max_text') && isValidFloat('ds18b20_min_text') &&
       isValidUint('rad_sensitivity_text')){
        gel('save-settings').disabled = false;
    }else{
        gel('save-settings').disabled = true;
    }
}
