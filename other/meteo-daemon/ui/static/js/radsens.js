let pstatic = [];
let pdynamic = [];

docReady(async function () {  
    getData(null);
});

function onRadsensChartNext(){
    onChartNext('period-radsens-chart', getData);
}

function onRadsensChartPrev(){
    onChartPrev('period-radsens-chart', getData);
}

function getData(end){
    let now = new Date();
    if(end != null){
        now = end;
    }

    pstatic = [];
    pdynamic = [];

    gel('radsens-cancas').style.display = 'none';
    showWaitWin('popup-wait');

    const cb = gel('period-radsens-chart');
    const select_id = cb.options[cb.selectedIndex].value;
    let period_end = now;
    let period_begin;
    let url;

    switch(select_id){
        case 'day':
            period_begin = subHours(now, 24);
            url = "/ui/radsens/day";
            checkNext('radsens-chart-next', secInDay(now));
            checkPrior('radsens-chart-ptev', secInDay(now));
            break;
        case 'week':
            period_begin = subDays(now, 7);
            url = "/ui/radsens/week";
            checkNext('radsens-chart-next', secInWeek(now));
            checkPrior('radsens-chart-ptev', secInWeek(now));
            break;
        case 'month':
            period_begin = subDays(now, 30);
            url = "/ui/radsens/month";
            checkNext('radsens-chart-next', secInMonth(now));
            checkPrior('radsens-chart-ptev', secInMonth(now));
            break;
        case 'year':
            period_begin = subMonth(now, 12);
            url = "/ui/radsens/year";
            checkNext('radsens-chart-next', secInYear(now));
            checkPrior('radsens-chart-ptev', secInYear(now));
            break;
    }

    let data = {
        end: period_end,
        begin: period_begin,
    };

    JsonPostMethod(url, data, success_radsens_data);
}

function success_radsens_data(json){   
    for(i=0; i < json.length; i++){
        if(json[i]["static"] != null && json[i]["dynamic"] != null){
            pstatic.push({x: simpleDate(json[i]["date_time"]), y: json[i]["static"]});    
            pdynamic.push({x: simpleDate(json[i]["date_time"]), y: json[i]["dynamic"]});    
        }
    }
    draw_radsens();
}

function draw_radsens(){
    const chart_id = gel("radsens-chart");
    let xTime = [];
    let yStatic = [];
    let yDynamic = [];

    let rev = 0;
    for(let i=pstatic.length-1; i >=0; i--){
        xTime[rev] = pstatic[i].x;
        yStatic[rev] = pstatic[i].y;
        yDynamic[rev] = pdynamic[i].y;
        rev=rev+1;
    }
 
    const options =  {
        legend: {display: false},
        spanGaps: false,
        scales: {
            y: {ticks: {min: 0, max:40}},
            x: {type: 'time', time: {unit: 'hour'}, display: true, ticks: {reverse: true}, gridLines: {display: false}}
        }
    }

    hideWaitWin('popup-wait');
    gel('radsens-cancas').style.display = 'block';

    clearCanvas();
    currentChart = new Chart(chart_id, {
        type: "line",
        data: {
            labels: xTime,
            datasets: [
                {fill: false, backgroundColor: "rgba(0,0,255,1.0)", borderColor: "blue", data: yStatic},
                {fill: false, backgroundColor: "rgba(0,0,255,1.0)", borderColor: "green", data: yDynamic}            
            ]
        },
        options: options
        });
}
