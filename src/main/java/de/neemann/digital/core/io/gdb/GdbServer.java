/*
 * Use of this source code is governed by the GPL v3 license
 * that can be found in the LICENSE file.
 */

package de.neemann.digital.core.io.gdb;

import de.neemann.digital.core.Model;
import de.neemann.digital.core.ModelEvent;
import de.neemann.digital.core.ModelEventType;
import de.neemann.digital.core.ModelStateObserverTyped;
import de.neemann.digital.core.Node;
import de.neemann.digital.core.NodeException;
import de.neemann.digital.core.ObservableValue;
import de.neemann.digital.core.ObservableValues;
import de.neemann.digital.core.element.Element;
import de.neemann.digital.core.element.ElementAttributes;
import de.neemann.digital.core.element.ElementTypeDescription;
import de.neemann.digital.core.element.Keys;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.concurrent.SynchronousQueue;
import java.util.concurrent.TimeUnit;

import static de.neemann.digital.core.element.PinInfo.input;

/**
 * GDB JTAG (remote_bitbang) Component
 * Sits in the schematic and toggles single-bit pins based on ASCII
 * characters received from OpenOCD. Automatically launches OpenOCD.
 */
public class GdbServer extends Node implements Element, ModelStateObserverTyped {

    /**
     * The description of the GdbServer element.
     */
    public static final ElementTypeDescription DESCRIPTION
            = new ElementTypeDescription(GdbServer.class, input("TDO"), input("C").setClock())
            .addAttribute(Keys.ROTATE)
            .addAttribute(Keys.LABEL);

    private final ObservableValue tckOut;
    private final ObservableValue tmsOut;
    private final ObservableValue tdiOut;
    private ObservableValue tdoIn;
    private ObservableValue clockIn;

    private boolean lastClock;
    private boolean currentTdo;
    private PinState activeState = null;

    // Latched states to hold pins steady between network packets
    private boolean latchedTck = false;
    private boolean latchedTms = false;
    private boolean latchedTdi = false;

    private final SynchronousQueue<PinState> pendingState = new SynchronousQueue<>();
    private final SynchronousQueue<Boolean> completedState = new SynchronousQueue<>();

    private volatile ServerSocket serverSocket;
    private volatile Socket activeClient;
    private Thread serverThread;
    private Process openocdProcess;
    private volatile boolean shuttingDown = false;

    /**
     * Creates a new GdbServer component.
     *
     * @param attributes the attributes of this element
     */
    public GdbServer(ElementAttributes attributes) {
        super(true);
        tckOut = new ObservableValue("TCK", 1).setPinDescription(DESCRIPTION);
        tmsOut = new ObservableValue("TMS", 1).setPinDescription(DESCRIPTION);
        tdiOut = new ObservableValue("TDI", 1).setPinDescription(DESCRIPTION);
    }

    @Override
    public void setInputs(ObservableValues inputs) throws NodeException {
        tdoIn = inputs.get(0).addObserverToValue(this);
        clockIn = inputs.get(1).addObserverToValue(this);

        if (serverThread == null) {
            serverThread = new Thread(this::runServer);
            serverThread.setDaemon(true);
            serverThread.setName("GdbServer-Server");
            serverThread.start();

            startOpenOCD();
        }
    }

    @Override
    public void init(Model model) throws NodeException {
        // Register so the framework tells us when the simulation is stopped/closed
        model.addObserver(this);
    }

    @Override
    public ModelEventType[] getEvents() {
        return new ModelEventType[]{ModelEventType.CLOSED};
    }

    @Override
    public void handleEvent(ModelEvent event) {
        if (event.getType() == ModelEventType.CLOSED)
            close();
    }

    @Override
    public ObservableValues getOutputs() {
        return new ObservableValues(tckOut, tmsOut, tdiOut);
    }

    /**
     * Closes the component, shutting down OpenOCD and the server socket.
     */
    public void close() {
        shuttingDown = true;

        if (openocdProcess != null) {
            openocdProcess.destroy();
            try {
                if (!openocdProcess.waitFor(1, TimeUnit.SECONDS)) {
                    openocdProcess.destroyForcibly();
                    openocdProcess.waitFor(1, TimeUnit.SECONDS);
                }
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                openocdProcess.destroyForcibly();
            }
        }

        if (activeClient != null) {
            try {
                activeClient.close();
            } catch (IOException ignored) {
            }
        }

        if (serverSocket != null) {
            try {
                serverSocket.close();
            } catch (IOException ignored) {
            }
        }

        if (serverThread != null) {
            serverThread.interrupt();
        }

        // Unblock any thread stuck waiting on the queues so it can exit cleanly
        pendingState.offer(new PinState(false, false, false));
        completedState.offer(true);
    }

    @Override
    public void readInputs() throws NodeException {
        boolean clock = clockIn.getBool();
        currentTdo = tdoIn.getBool(); // continually track TDO for immediate 'R' responses

        if (!clock && lastClock) { // Falling edge processing
            if (activeState != null) {
                try {
                    // Use timeout to prevent dropping tokens if threads desync
                    if (!completedState.offer(true, 100, TimeUnit.MILLISECONDS)) {
                        System.err.println("GdbServer: server thread not consuming completed states");
                    }
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
                activeState = null;
            }
            activeState = pendingState.poll();

            // Latch the requested pins so they hold steady between OpenOCD commands
            if (activeState != null) {
                latchedTck = activeState.tck;
                latchedTms = activeState.tms;
                latchedTdi = activeState.tdi;
            }
        }
        lastClock = clock;
    }

    @Override
    public void writeOutputs() throws NodeException {
        // Output latched states. NEVER default to 0, or you destroy the bitbang waveform!
        tckOut.setValue(latchedTck ? 1 : 0);
        tmsOut.setValue(latchedTms ? 1 : 0);
        tdiOut.setValue(latchedTdi ? 1 : 0);
    }

    private void startOpenOCD() {
        try {
            ProcessBuilder pb = new ProcessBuilder(
                    "openocd",
                    "-c", "adapter driver remote_bitbang",
                    "-c", "remote_bitbang port 9824",
                    "-c", "remote_bitbang host 127.0.0.1",
                    "-c", "adapter speed 10000",
                    "-c", "set _CHIPNAME riscv",
                    "-c", "jtag newtap $_CHIPNAME cpu -irlen 5 -expected-id 0x01",
                    "-c", "set _TARGETNAME $_CHIPNAME.cpu",
                    "-c", "target create $_TARGETNAME riscv -chain-position $_TARGETNAME",
                    "-c", "gdb_port 3333",
                    "-c", "init",
                    "-c", "halt",
                    "-d3"
            );

            // Redirects OpenOCD's stdout/stderr to your Java console
            pb.inheritIO();

            openocdProcess = pb.start();
            System.out.println("GdbServer: Launched OpenOCD child process with inline RISC-V config.");
        } catch (IOException e) {
            System.err.println("GdbServer: Failed to start OpenOCD: " + e.getMessage());
        }
    }

    private void runServer() {
        try {
            serverSocket = new ServerSocket(9824);
            System.out.println("GdbServer: Listening on port 9824...");

            while (!Thread.currentThread().isInterrupted()) {
                try {
                    activeClient = serverSocket.accept();
                    System.out.println("GdbServer: OpenOCD connected.");

                    InputStream in = activeClient.getInputStream();
                    OutputStream out = activeClient.getOutputStream();

                    int c;
                    while ((c = in.read()) != -1) {
                        char cmd = (char) c;

                        switch (cmd) {
                            case 'B': case 'b': case 'r':
                                break;
                            case '0': case '1': case '2': case '3':
                            case '4': case '5': case '6': case '7':
                                int val = cmd - '0';
                                boolean tdi = (val & 1) != 0;
                                boolean tms = ((val >> 1) & 1) != 0;
                                boolean tck = ((val >> 2) & 1) != 0;

                                // Queue state and block until schematic ticks it
                                pendingState.put(new PinState(tck, tms, tdi));
                                completedState.take();
                                break;
                            case 'R':
                                // Read TDO asynchronously from last known state
                                out.write(currentTdo ? '1' : '0');
                                out.flush();
                                break;
                            case 'Q':
                                System.out.println("GdbServer: Quit request.");
                                return;
                        }
                    }
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
                } catch (Exception e) {
                    if (shuttingDown || (serverSocket != null && serverSocket.isClosed())) {
                        // Expected: we closed the socket ourselves during shutdown
                        break;
                    }
                    System.err.println("GdbServer: Client disconnected or error.");
                } finally {
                    if (activeClient != null) {
                        try {
                            activeClient.close();
                        } catch (IOException ignored) {
                        }
                    }
                }
            }
        } catch (Exception e) {
            if (!shuttingDown)
                e.printStackTrace();
        } finally {
            System.out.println("GdbServer: Server thread exiting.");
        }
    }

    private static final class PinState {
        final boolean tck;
        final boolean tms;
        final boolean tdi;
        PinState(boolean tck, boolean tms, boolean tdi) {
            this.tck = tck;
            this.tms = tms;
            this.tdi = tdi;
        }
    }
}
