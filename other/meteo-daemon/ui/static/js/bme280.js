let pressure = [];
let temperature = [];
let humidity = [];

docReady(async function () {  
    getData(null);
});


function onBme280ChartNext(){
    onChartNext('period-bme280-chart', getData);
}

function onBme280ChartPrev(){
    onChartPrev('period-bme280-chart', getData);
}

function getBmp280Gradation(){
    const cb= gel('bme280-gradation');
    const grad_id = cb.options[cb.selectedIndex].value;

    let ret = '';

    switch(grad_id){
        case 'bmp280-max':
            ret = 'max';
            break;
        case 'bmp280-min':
            ret = 'min';
            break;
        default:
            ret = 'avg';
    }
    return ret;
}

function getData(end){  
    let now = new Date();
    if(end != null){
        now = end;
    }

    pressure = [];
    temperature = [];
    humidity = [];

    gel('bme280-cancas').style.display = 'none';
    showWaitWin('popup-wait');

    const cb = gel('period-bme280-chart');
    const select_id = cb.options[cb.selectedIndex].value;
    let period_end = now;
    let period_begin;
    let url;

    switch(select_id){
        case 'day':
            period_begin = subHours(now, 24);
            url = "/ui/bmx280/day/" + getBmp280Gradation();
            checkNext('bme280-chart-next', secInDay(now));
            checkPrior('bme280-chart-ptev', secInDay(now));
            break;
        case 'week':
            period_begin = subDays(now, 7);
            url = "/ui/bmx280/week/" + getBmp280Gradation();
            checkNext('bme280-chart-next', secInWeek(now));
            checkPrior('bme280-chart-ptev', secInWeek(now));
            break;
        case 'month':
            period_begin = subMonth(now, 1);
            url = "/ui/bmx280/month/" + getBmp280Gradation();
            checkNext('bme280-chart-next', secInMonth(now));
            checkPrior('bme280-chart-ptev', secInMonth(now));
            break;
        case 'year':
            period_begin = subMonth(now, 12);
            url = "/ui/bmx280/year/" + getBmp280Gradation();
            checkNext('bme280-chart-next', secInYear(now));
            checkPrior('bme280-chart-ptev', secInYear(now));
            break;
    }

    let data = {
        end: period_end,
        begin: period_begin,
    };

    JsonPostMethod(url, data, success_bme280_data)
}

function success_bme280_data(json){
    for(i=0; i < json.length; i++){ 
        //console.log("Press: "+json[i]["press"]+", Tempr: "+json[i]["tempr"], + ", Hum: "+ json[i]["hum"] + ", Date: "+json[i]["date_time"]);
        if(json[i]["press"] != null){
            pressure.push({x: simpleDate(json[i]["date_time"]), y: json[i]["press"]});    
        }
        if(json[i]["tempr"] != null){
            temperature.push({x: simpleDate(json[i]["date_time"]), y: json[i]["tempr"]}); 
        }
        if(json[i]["hum"] != null){  
            humidity.push({x: simpleDate(json[i]["date_time"]), y: json[i]["hum"]});
        }
    }
    draw_bme280();
}

function draw_bme280(){
    const el = gel('show-bme280-chart');
    const show = el.options[el.selectedIndex].value;

    switch(show){
        case 'pressure':
            draw_bme280_pressure();
            break;
        case 'temperature':
            draw_bme280_temperature();
            break;
        case 'humidity':
            draw_bme280_humidity();
            break;
    }
}

function draw_bme280_pressure(){
    const chart_id = gel("bme280-chart");
    let xTime = [];
    let yPressure = [];

    let rev = 0;
    for(let i=pressure.length-1; i >=0; i--){
        xTime[rev] = pressure[i].x;
        yPressure[rev] = fixeFloat(pressure[i].y/133, 1);
        rev=rev+1;
    }
    
    const options =  {
        legend: {display: false},
        spanGaps: false,
        scales: {
            y: {ticks: {min: 600, max: 850}},
            x: {type: 'time', time: {unit: 'hour'}, display: true, ticks: {reverse: true}, gridLines: {display: false}}
        }
    }

    hideWaitWin('popup-wait');
    gel('bme280-cancas').style.display = 'block';

    clearCanvas();
    currentChart = new Chart(chart_id, {
        type: "line",
        data: {
            labels: xTime,
            datasets: [
                {fill: false, backgroundColor: "rgba(0,0,255,1.0)", borderColor: "blue", data: yPressure}
            ]
        },
        options: options
        });
}

function draw_bme280_temperature(){
    const chart_id = gel("bme280-chart");
    let xTime = [];
    let yTemperature = [];

    let rev = 0;
    for(let i=temperature.length-1; i >=0; i--){
        xTime[rev] = temperature[i].x;
        yTemperature[rev] = temperature[i].y;
        rev=rev+1;
    }
    
    const options =  {
        legend: {display: false},
        spanGaps: false,
        scales: {
            y: {ticks: {min: -50, max: +50}},
            x: {type: 'time', time: {unit: 'hour'}, display: true, ticks: {reverse: true}, gridLines: {display: false}}
        }
    }

    hideWaitWin('popup-wait');
    gel('bme280-cancas').style.display = 'block';

    clearCanvas();
    currentChart = new Chart(chart_id, {
        type: "line",
        data: {
            labels: xTime,
            datasets: [
                {fill: false, backgroundColor: "rgba(0,0,255,1.0)", borderColor: "blue", data: yTemperature}
            ]
        },
        options: options
        });
}

function draw_bme280_humidity(){
    const chart_id = gel("bme280-chart");
    let xTime = [];
    let yHumidity = [];

    let rev = 0;
    for(let i=humidity.length-1; i >=0; i--){
        xTime[rev] = humidity[i].x;
        yHumidity[rev] = humidity[i].y;
        rev=rev+1;
    }
    
    const options =  {
        legend: {display: false},
        spanGaps: false,
        scales: {
            y: {ticks: {min: 0, max: 100}},
            x: {type: 'time', time: {unit: 'hour'}, display: true, ticks: {reverse: true}, gridLines: {display: false}}
        }
    }

    hideWaitWin('popup-wait');
    gel('bme280-cancas').style.display = 'block';

    clearCanvas();
    currentChart = new Chart(chart_id, {
        type: "line",
        data: {
            labels: xTime,
            datasets: [
                {fill: false, backgroundColor: "rgba(0,0,255,1.0)", borderColor: "blue", data: yHumidity}
            ]
        },
        options: options
        });
}
