/* eslint-disable @typescript-eslint/no-unused-expressions */
import { app, BrowserWindow, ipcMain, protocol, shell, session } from "electron";
import { join } from "path";
import installExtension, { VUEJS_DEVTOOLS } from "electron-devtools-installer";
import { productName } from "../package.json";
import createProtocol from "./createProtocol";

const isDevelopment = process.env.NODE_ENV === "development";

// Scheme must be registered before the app is ready
protocol.registerSchemesAsPrivileged([{ scheme: "app", privileges: { secure: true, standard: true } }]);

/**
 * ** The built directory structure
 * ------------------------------------
 * ├─┬ build/electron
 * │ └── main.js        > Electron-Main
 * │ └── preload.js     > Preload-Scripts
 * ├─┬ build/app
 *   └── index.html     > Electron-Renderer
 */

process.env.BUILD_APP = join(__dirname, "../app");
process.env.PUBLIC = process.env.VITE_DEV_SERVER_URL ? join(__dirname, "../../public") : process.env.BUILD_APP;

let mainWindow: BrowserWindow | null = null;

function createWindow() {
  mainWindow = new BrowserWindow({
    icon: join(process.env.PUBLIC, "icon.png"),
    title: `${productName}${isDevelopment ? " [development]" : ""}`,
    width: 1024,
    height: 600,
    resizable: true,
    maximizable: true,
    webPreferences: {
      // Warning: Enable nodeIntegration and disable contextIsolation is not secure in production
      // Consider using contextBridge.exposeInMainWorld
      // Read more on https://www.electronjs.org/docs/latest/tutorial/context-isolation
      nodeIntegration: false,
      contextIsolation: true,
      sandbox: !isDevelopment,
      preload: join(__dirname, "preload.js"),
    },
  });

  if (process.env.VITE_DEV_SERVER_URL) {
    mainWindow.loadURL(process.env.VITE_DEV_SERVER_URL);
  } else {
    createProtocol("app");
    mainWindow.loadFile(join(process.env.BUILD_APP, "index.html"));
  }

  if (isDevelopment) {
    mainWindow.webContents.openDevTools();
    mainWindow.maximize();
  } else {
    // No menu bar in production
    mainWindow.removeMenu();
  }

  // Make all links open with the browser, not with the application
  mainWindow.webContents.setWindowOpenHandler(({ url }) => {
    if (url.startsWith("https:")) shell.openExternal(url);
    return { action: "deny" };
  });

  if (!process.env.NO_FULLSCREEN) {
    mainWindow.setFullScreen(true);
  }
}

// This method will be called when Electron has finished
// initialization and is ready to create browser windows.
// Some APIs can only be used after this event occurs.
app.on("ready", async () => {
  ipcMain.handle("host", () => {
    const argv = process.argv.slice(0);
    // Get the host argument from command line
    const hostArg = argv.find((arg) => arg.startsWith("--host="));
    let host: string;
    if (hostArg) {
      host = hostArg.split("=")[1];
    }
    return host;
  });

  if (isDevelopment) {
    // Install Vue Devtools
    try {
      await installExtension(VUEJS_DEVTOOLS);
    } catch (e) {
      console.error("Vue Devtools failed to install:", e.toString());
    }
  }

  createWindow();

  // mainWindow.once("ready-to-show", () => {
  // 	mainWindow.show();
  // });

  // header security policy
  session.defaultSession.webRequest.onHeadersReceived((details, callback) => {
    callback({
      responseHeaders: {
        ...details.responseHeaders,
        "Content-Security-Policy": [
          isDevelopment
            ? `script-src 'self' 'sha256-3bzWVxQE32IZQKH9eh8KzyHuhXOlMrboDVVBRd0fWTU='` // Allow devtools in dev
            : "script-src 'self'", // Strict in production
        ],
      },
    });
  });
});

app.on("activate", () => {
  // On macOS, it's common to re-create a window in the app when the
  // dock icon is clicked and there are no other windows open.
  const allWindows = BrowserWindow.getAllWindows();
  allWindows.length === 0 ? createWindow() : allWindows[0].focus();
});

// Quit when all windows are closed, except on macOS. There, it's common
// for applications and their menu bar to stay active until the user quits
// explicitly with Cmd + Q.
app.on("window-all-closed", () => {
  mainWindow = null;
  if (process.platform !== "darwin") app.quit();
});

// In this file you can include the rest of your app's specific main process
// code. You can also put them in separate files and require them here.
