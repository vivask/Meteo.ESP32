docReady(async function () {  
    getData(null);
});

function onLogingRefresh(){
    getData(currentDateTime);
}

function onLogingClear(){
    let answer = window.confirm("Удалить все записи в журнале?");
    if(answer){
        JsonPutMethod("/verify/loging/clear", null, success_reload)
    }    
}

async function getData(end){
    let now = new Date();
    if(end != null){
        now = end;
    }

    clearTable('tbl-loging');

    const cb = gel('cb-loging');
    const period = cb.options[cb.selectedIndex].value;
    let period_end = now;
    let period_begin = period_end;

    switch(period){
        case 'day':
            period_begin = subDays(now, 1);
            checkNext('loging-next', secInDay(now));
            checkPrior('loging-ptev', secInDay(now));
            break;
        case 'week':
            period_begin = subDays(now, 7);
            checkNext('loging-next', secInWeek(now));
            checkPrior('loging-ptev', secInWeek(now));
            break;
        case 'month':
            period_begin = subMonth(now, 1);
            checkNext('loging-next', secInMonth(now));
            checkPrior('loging-ptev', secInMonth(now));
            break;
        case 'year':
            period_begin = subYear(now, 1);
            checkNext('loging-next', secInYear(now));
            checkPrior('loging-ptev', secInYear(now));
            break;
    }

    let data = {
        end: period_end,
        begin: period_begin,
    };

    JsonPostMethod("/verify/loging/get", data, success_loging_data)
}

function getColor(type){
    let color = 'white';
    switch(type){
        case 'I':
            color = 'green';
            break;
        case 'W':
            color = 'blue';
            break;
        case 'E':
            color = 'red';
            break;
    }
    return color;
}

function success_loging_data(json){
    let tbl = gel('tbl-loging');      
    for(let i=0; i < json.length; i++){         
        const tr = tbl.insertRow();
        let td = tr.insertCell();
        td.style.border = '1px solid black';
        td.style.width = '100px';
        td.style.padding = '5px';
        td.style.align = 'center';
        td.style.color = getColor(json[i]["type"]);
        td.appendChild(document.createTextNode(json[i]["date"]));
        td = tr.insertCell();
        td.style.border = '1px solid black';
        td.style.width = '80px';
        td.style.padding = '5px';
        td.style.color = getColor(json[i]["type"]);
        td.appendChild(document.createTextNode(json[i]["time"]));
        td = tr.insertCell();
        td.style.border = '1px solid black';
        td.style.color = getColor(json[i]["type"]);
        td.style.padding = '5px';
        td.appendChild(document.createTextNode(json[i]["message"]));
    }
}

function doCBLogingChange(){
    getData(currentDateTime);
}

function onLogingPrev(){
    onChartPrev('cb-loging', getData);
}

function onLogingNext(){
    onChartNext('cb-loging', getData);
}
