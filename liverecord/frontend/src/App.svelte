<script lang="ts">
  import { io } from "socket.io-client";

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
  }

  $: chartConfigs = computeChartConfigs(state)
   
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

<h1>{JSON.stringify(state)}</h1>

<style>
  .logo {
    height: 6em;
    padding: 1.5em;
    will-change: filter;
  }
  .logo:hover {
    filter: drop-shadow(0 0 2em #646cffaa);
  }
  .logo.svelte:hover {
    filter: drop-shadow(0 0 2em #ff3e00aa);
  }
  .read-the-docs {
    color: #888;
  }
</style>