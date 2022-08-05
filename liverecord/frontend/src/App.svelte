<script lang="ts">
  import svelteLogo from './assets/svelte.svg'
  import Counter from './lib/Counter.svelte'

  let count = "Loading...";
  fetch("http://localhost:5000", {
    headers: [
      ["Access-Control-Allow-Origin", "http://localhost:5000"]
    ]
  }).then((val) => {
    val.text().then((text) => {
      count = text
    })
  })
</script>

<h1>{count}!</h1>

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