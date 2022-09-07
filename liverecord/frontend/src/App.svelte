<script lang="ts">
  import { io } from "socket.io-client";
import highcharts from "./highcharts";

  let state: { [index: string]: Array<Array<number>> } = {}
  
  function computeChartConfigs(chartDatas: { [index: string]: Array<Array<number>> }) {
    let configs = []
    for (const chart of Object.entries(chartDatas)) {
      configs.push({
        chart: {
          type: 'spline'
        },
        title: {
          text: chart[0]
        },
        xAxis: {
          title: {
            text: 'X'
          }
        },
        yAxis: {
          title: {
            text: 'Y'
          }
        },
        series: [
          {
            name: chart[0],
            data: chart[1]
          }
        ]
      })
    }
    return configs
  }

  $: chartConfigs = computeChartConfigs(state)
  $: {
    console.log("Something is being modified.")
  }
  const socket = io("http://localhost:5000")
  socket.on("fullPayload", (args) => {
    state = args
  })

  socket.on("newValue", (args) => {
    const toModify = state
    toModify[args["key"]].push(args["value"])
    state = toModify
  })
</script>


{#each chartConfigs as config}
  <div class="chart" use:highcharts={config}></div>  
{/each}
