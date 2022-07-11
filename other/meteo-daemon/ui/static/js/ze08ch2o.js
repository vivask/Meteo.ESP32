let points = [];

docReady(async function () {  
    getData(null);
});

function onZeChartNext(){
    onChartNext('period-ze08-chart', getData);
}

function onZeChartPrev(){
    onChartPrev('period-ze08-chart', getData);
}

function getData(end){
    let now = new Date();
    if(end != null){
        now = end;
    }

    points = [];

    gel('ze08ch20-cancas').style.display = 'none';
    showWaitWin('popup-wait');

    const cb = gel('period-ze08-chart');
    const select_id = cb.options[cb.selectedIndex].value;
    let period_end = now;
    let period_begin;
    let url;

    switch(select_id){
        case 'day':
            period_begin = subHours(now, 24);
            url = "/ui/ze08ch2o/day";
            checkNext('ze-chart-next', secInDay(now));
            checkPrior('ze-chart-ptev', secInDay(now));
            break;
        case 'week':
            period_begin = subDays(now, 7);
            url = "/ui/ze08ch2o/week";
            checkNext('ze-chart-next', secInWeek(now));
            checkPrior('ze-chart-ptev', secInWeek(now));
            break;
        case 'month':
            period_begin = subDays(now, 30);
            url = "/ui/ze08ch2o/month";
            checkNext('ze-chart-next', secInMonth(now));
            checkPrior('ze-chart-ptev', secInMonth(now));
            break;
        case 'year':
            period_begin = subMonth(now, 12);
            url = "/ui/ze08ch2o/year";
            checkNext('ze-chart-next', secInYear(now));
            checkPrior('ze-chart-ptev', secInYear(now));
            break;
    }

    let data = {
        end: period_end,
        begin: period_begin,
    };

    JsonPostMethod(url, data, success_ze08ch20_data);
}

function success_ze08ch20_data(json){  
    for(i=0; i < json.length; i++){ 
        if(json[i]["ch2o"] != null){
            points.push({x: simpleDate(json[i]["date_time"]), y: json[i]["ch2o"]});
        }
    }
    draw_ze08ch20();
}

function draw_ze08ch20(){
    const chart_id = gel("ze08ch20-chart");
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
            y: {ticks: {min: 10, max:400}},
            x: {type: 'time', time: {unit: 'hour'}, display: true, ticks: {reverse: true}, gridLines: {display: false}}
        }
    }

    hideWaitWin('popup-wait');
    gel('ze08ch20-cancas').style.display = 'block';

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
