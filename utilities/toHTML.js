const { exec } = require("child_process");
const fs = require("fs");
const { join, relative } = require("path");

let output = "";

const toHTML = async (path) => {
  const stat = fs.statSync(path);

  if (stat.isFile()) {
    // run pygmentize -l cpp -f html -O full,linenos=1,style=default ~/Documents/Code/School/2022/Vex\ Robotics/src/main.cpp
    const data = exec(
      `pygmentize -l cpp -f html -O full,linenos=1,style=default "${path}"`,
    );

    const relativePath = relative(
      join(process.cwd(), ".."),
      path
    );

    let commandOutput = "<hr /><h1>File: " + relativePath + "</h1>\n";

    console.log(`Running ${path}...`)

    return new Promise((resolve) => {

      data.stdout.on("data", (data) => {
        commandOutput += data;
      });

      data.on("close", () => {
        console.log(`Finished ${path}`);
        output += commandOutput;
        resolve();
      });

    });
  } else {
    const files = fs.readdirSync(path);

    return Promise.all(
      files.map((file) => toHTML(join(path, file)))
    );

  }
};

// run on current working directory
toHTML(process.cwd()).then(() => {
  // write to file
  fs.writeFileSync("../output.html", output);
});
