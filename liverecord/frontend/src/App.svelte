<script lang="ts">
  import { io } from "socket.io-client";

  let text = "Loading..."
  let count = 0
  
  const socket = io("http://localhost:5000")
  socket.on("Testing", (args) => {
    text = "SOMETHING HAPPENED!"
  })
</script>

<h1>{text}</h1>
<button on:click={() => { count = count + 1}}>{count}</button>
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