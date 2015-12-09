if (process.argv.length < 3)
    console.log("Usage: " + process.argv[0] + " " + process.argv[1] + " ADDRESS");
else
    require('./lib/aira_bridge')(process.argv[2])
