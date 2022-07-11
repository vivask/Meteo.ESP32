let points = [];

docReady(async function () {  
    getData(null);
});

function onDS18B20ChartNext(){
    onChartNext('period-ds18b20-chart', getData);
}

function onDS18B20ChartPrev(){
    onChartPrev('period-ds18b20-chart', getData);
}

function getData(end){
    let now = new Date();
    if(end != null){
        now = end;
    }

    points = [];

    gel('ds18b20-cancas').style.display = 'none';
    showWaitWin('popup-wait');

    const cb = gel('period-ds18b20-chart');
    const select_id = cb.options[cb.selectedIndex].value;
    let period_end = now;
    let period_begin;
    let url;

    switch(select_id){
        case 'day':
            period_begin =subHours(now, 24);
            url = "/ui/ds18b20/day";
            checkNext('ds18b20-chart-next', secInDay(now));
            checkPrior('ds18b20-chart-ptev', secInDay(now));
            break;
        case 'week':
            period_begin = subDays(now, 7);
            url = "/ui/ds18b20/week";
            checkNext('ds18b20-chart-next', secInWeek(now));
            checkPrior('ds18b20-chart-ptev', secInWeek(now));
            break;
        case 'month':
            period_begin = subMonth(now, 1);
            url = "/ui/ds18b20/month";
            checkNext('ds18b20-chart-next', secInMonth(now));
            checkPrior('ds18b20-chart-ptev', secInMonth(now));
            break;
        case 'year':
            period_begin = subMonth(now, 12);
            url = "/ui/ds18b20/year";
            checkNext('ds18b20-chart-next', secInYear(now));
            checkPrior('ds18b20-chart-ptev', secInYear(now));
            break;
    }    
    let data = {
        end: period_end,
        begin: period_begin,
    };

    JsonPostMethod(url, data, success_ds18b20_data);

}


function success_ds18b20_data(json){   
    for(i=0; i < json.length; i++){
        if(json[i]["tempr"] != null){
            points.push({x: simpleDate(json[i]["date_time"]), y: json[i]["tempr"]});    
        }
    }
    draw_ds18b20();
}

function draw_ds18b20(){
    const chart_id = gel("ds18b20-chart");
    let xValues = [];
    let yValues = [];

    let rev = 0;
    for(let i=points.length-1; i >=0; i--){
        xValues[rev] = points[i].x;
        yValues[rev] = points[i].y;
        rev=rev+1;
    }
 
    const options =  {
        legend: {display: false},
        spanGaps: false,
        scales: {
            y: {ticks: {min: 0, max:50}},
            x: {type: 'time', time: {unit: 'hour'}, display: true, ticks: {reverse: true}, gridLines: {display: false}}
        }
    }

    hideWaitWin('popup-wait');
    gel('ds18b20-cancas').style.display = 'block';

    clearCanvas();
    currentChart = new Chart(chart_id, {
        type: "line",
        data: {
            labels: xValues,
            datasets: [{
            fill: false,
            backgroundColor: "rgba(0,0,255,1.0)",
            borderColor: "blue",
            data: yValues
            }]
        },
        options: options
        });
}