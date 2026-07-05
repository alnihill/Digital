/*
 * Use of this source code is governed by the GPL v3 license
 * that can be found in the LICENSE file.
 */

package de.neemann.digital.gui.components.gdb;

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
import java.net.SocketException;
import java.net.InetSocketAddress;
import java.util.concurrent.SynchronousQueue;
import java.util.concurrent.TimeUnit;

import static de.neemann.digital.core.element.PinInfo.input;

/**
 * A GDB server component that bridges GDB's remote serial protocol to JTAG pins.
 *
 * <p>Architecture overview:
 * <pre>
 * GDB client  <--TCP-->  GDB server thread  <--BitRequest queues-->  Simulator thread
 * </pre>
 */
public class Gdb extends Node implements Element {

    /**
     * The component description.
     */
    public static final ElementTypeDescription DESCRIPTION
            = new ElementTypeDescription(Gdb.class, input("TDO"), input("C").setClock())
            .addAttribute(Keys.ROTATE)
            .addAttribute(Keys.LABEL);

    private static final int BASE_GDB_PORT = 1234;
    private static final int MAX_GDB_PORT  = 1250;

    // -------------------------------------------------------------------------
    // JTAG / RISC-V Debug Module constants
    // -------------------------------------------------------------------------

    private static final int  IR_BITS        = 5;
    private static final long IR_DTMCS       = 0x10L;
    private static final long IR_DMI         = 0x11L;

    private static final int  DMI_ABITS      = 7;
    private static final int  DMI_TOTAL_BITS = DMI_ABITS + 32 + 2;

    private static final int  DMI_OP_NOP   = 0;
    private static final int  DMI_OP_READ  = 1;
    private static final int  DMI_OP_WRITE = 2;

    private static final int  DMI_STATUS_SUCCESS = 0;
    private static final int  DMI_STATUS_FAILED  = 2;
    private static final int  DMI_STATUS_BUSY    = 3;

    private static final long DTMCS_DMIRESET     = (1L << 16);

    // RISC-V Debug Module register addresses
    private static final long DMI_DATA0      = 0x04L;
    private static final long DMI_DMCONTROL  = 0x10L;
    private static final long DMI_DMSTATUS   = 0x11L;
    private static final long DMI_ABSTRACTCS = 0x16L;
    private static final long DMI_COMMAND    = 0x17L;
    private static final long DMI_PROGBUF0   = 0x20L;
    private static final long DMI_PROGBUF1   = 0x21L;
    private static final long DMI_PROGBUF2   = 0x22L;
    private static final long DMI_PROGBUF3   = 0x23L;
    private static final long DMI_PROGBUF4   = 0x24L;
    private static final long DMI_PROGBUF5   = 0x25L;
    private static final long DMI_PROGBUF6   = 0x26L;
    private static final long DMI_PROGBUF7   = 0x27L;

    // Abstract Command constants
    private static final long CMD_POSTEXEC      = (1L << 18); // Executes progbuf without reg transfer

    private static final long DMCONTROL_DMACTIVE   = 1L;
    private static final long DMCONTROL_RESUMEREQ  = (1L << 30);
    private static final long DMCONTROL_HALT_REQ   = (1L << 31);
    private static final long DMSTATUS_ALLHALTED   = (1L << 9);
    private static final long DMSTATUS_ALLRESUMEACK = (1L << 17);

    // -------------------------------------------------------------------------
    // Pre-encoded RISC-V instructions for progbuf sequences
    // -------------------------------------------------------------------------

    private static final long INSN_CSRW_DSCRATCH0_X10 = 0x7b251073L;
    private static final long INSN_CSRW_DSCRATCH1_X11 = 0x7b359073L;
    private static final long INSN_LUI_X11_1          = 0x000015b7L;
    private static final long INSN_CSRR_X10_DSCRATCH0 = 0x7b202573L;
    private static final long INSN_CSRR_X11_DSCRATCH1 = 0x7b3025f3L;
    private static final long INSN_EBREAK             = 0x00100073L;

    private static final long INSN_CSRR_X10_DPC = 0x7b102573L;
    private static final long INSN_CSRW_DPC_X10 = 0x7b151073L;

    private static final long INSN_CSRR_X10_DCSR = 0x7b002573L;
    private static final long INSN_CSRW_DCSR_X10 = 0x7b051073L;

    private static final long INSN_SW_BASE = 0x0205a223L;
    private static final long INSN_LW_BASE = 0x0245a003L;

    private static final long INSN_SB_X10_X11  = 0x00a58023L;
    private static final long INSN_LBU_X10_X11 = 0x00058503L;

    // -------------------------------------------------------------------------
    // Target XML (RISC-V RV32I)
    // -------------------------------------------------------------------------
    private static final String TARGET_XML
            = "<?xml version=\"1.0\"?>"
            + "<!DOCTYPE target SYSTEM \"gdb-target.dtd\">"
            + "<target>"
            + "<architecture>riscv:rv32</architecture>"
            + "<feature name=\"org.gnu.gdb.riscv.cpu\">"
            + "<reg name=\"x0\"  bitsize=\"32\" type=\"int\"/>"
            + "<reg name=\"x1\"  bitsize=\"32\" type=\"int\"/>"
            + "<reg name=\"x2\"  bitsize=\"32\" type=\"int\"/>"
            + "<reg name=\"x3\"  bitsize=\"32\" type=\"int\"/>"
            + "<reg name=\"x4\"  bitsize=\"32\" type=\"int\"/>"
            + "<reg name=\"x5\"  bitsize=\"32\" type=\"int\"/>"
            + "<reg name=\"x6\"  bitsize=\"32\" type=\"int\"/>"
            + "<reg name=\"x7\"  bitsize=\"32\" type=\"int\"/>"
            + "<reg name=\"x8\"  bitsize=\"32\" type=\"int\"/>"
            + "<reg name=\"x9\"  bitsize=\"32\" type=\"int\"/>"
            + "<reg name=\"x10\" bitsize=\"32\" type=\"int\"/>"
            + "<reg name=\"x11\" bitsize=\"32\" type=\"int\"/>"
            + "<reg name=\"x12\" bitsize=\"32\" type=\"int\"/>"
            + "<reg name=\"x13\" bitsize=\"32\" type=\"int\"/>"
            + "<reg name=\"x14\" bitsize=\"32\" type=\"int\"/>"
            + "<reg name=\"x15\" bitsize=\"32\" type=\"int\"/>"
            + "<reg name=\"x16\" bitsize=\"32\" type=\"int\"/>"
            + "<reg name=\"x17\" bitsize=\"32\" type=\"int\"/>"
            + "<reg name=\"x18\" bitsize=\"32\" type=\"int\"/>"
            + "<reg name=\"x19\" bitsize=\"32\" type=\"int\"/>"
            + "<reg name=\"x20\" bitsize=\"32\" type=\"int\"/>"
            + "<reg name=\"x21\" bitsize=\"32\" type=\"int\"/>"
            + "<reg name=\"x22\" bitsize=\"32\" type=\"int\"/>"
            + "<reg name=\"x23\" bitsize=\"32\" type=\"int\"/>"
            + "<reg name=\"x24\" bitsize=\"32\" type=\"int\"/>"
            + "<reg name=\"x25\" bitsize=\"32\" type=\"int\"/>"
            + "<reg name=\"x26\" bitsize=\"32\" type=\"int\"/>"
            + "<reg name=\"x27\" bitsize=\"32\" type=\"int\"/>"
            + "<reg name=\"x28\" bitsize=\"32\" type=\"int\"/>"
            + "<reg name=\"x29\" bitsize=\"32\" type=\"int\"/>"
            + "<reg name=\"x30\" bitsize=\"32\" type=\"int\"/>"
            + "<reg name=\"x31\" bitsize=\"32\" type=\"int\"/>"
            + "<reg name=\"pc\"  bitsize=\"32\" type=\"code_ptr\"/>"
            + "</feature>"
            + "<feature name=\"org.gnu.gdb.riscv.csr\">"
            + "<reg name=\"dpc\" bitsize=\"32\" type=\"code_ptr\" regnum=\"6065\"/>"
            + "</feature>"
            + "</target>";

    // -------------------------------------------------------------------------
    // Simulator-side state
    // -------------------------------------------------------------------------
    private final ObservableValue tckOut;
    private final ObservableValue tmsOut;
    private final ObservableValue tdiOut;
    private ObservableValue tdoIn;
    private ObservableValue clockIn;
    private boolean lastClock;
    private BitRequest activeBit = null;

    private final SynchronousQueue<BitRequest> pendingBit   = new SynchronousQueue<>();
    private final SynchronousQueue<BitRequest> completedBit = new SynchronousQueue<>();

    private volatile ServerSocket serverSocket;
    private volatile Socket activeClient;
    private Thread serverThread;

    /**
     * Creates a new GDB server instance.
     *
     * @param attributes the component attributes
     */
    public Gdb(ElementAttributes attributes) {
        tckOut = new ObservableValue("TCK", 1).setPinDescription(DESCRIPTION);
        tmsOut = new ObservableValue("TMS", 1).setPinDescription(DESCRIPTION);
        tdiOut = new ObservableValue("TDI", 1).setPinDescription(DESCRIPTION);
    }

    @Override
    public void setInputs(ObservableValues inputs) throws NodeException {
        tdoIn   = inputs.get(0).addObserverToValue(this);
        clockIn = inputs.get(1).addObserverToValue(this);

        if (serverThread == null) {
            serverThread = new Thread(this::runGdbServer);
            serverThread.setDaemon(true);
            serverThread.setName("GDB-Server");
            serverThread.start();
        }
    }

    @Override
    public ObservableValues getOutputs() {
        return new ObservableValues(tckOut, tmsOut, tdiOut);
    }

    /**
     * Closes the GDB server socket and thread.
     */
    public void close() {
        ServerSocket ss = serverSocket;
        if (ss != null) {
            try {
                ss.close();
            } catch (IOException ignored) {

            }
        }

        Socket ac = activeClient;
        if (ac != null) {
            try {
                ac.close();
            } catch (IOException ignored) {

            }
        }

        if (serverThread != null) {
            serverThread.interrupt();
        }
    }

    @Override
    public void readInputs() throws NodeException {
        boolean clock = clockIn.getBool();

        if (!clock && lastClock) {
            if (activeBit != null) {
                activeBit.setTdo((tdoIn != null) && tdoIn.getBool());
                try {
                    if (!completedBit.offer(activeBit, 100, TimeUnit.MILLISECONDS)) {
                        System.err.println("GDB: server thread not consuming completed bits");
                    }
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
                activeBit = null;
            }
            activeBit = pendingBit.poll();
        }
        lastClock = clock;
    }

    @Override
    public void writeOutputs() throws NodeException {
        if (activeBit != null) {
            tmsOut.setValue(activeBit.tms ? 1 : 0);
            tdiOut.setValue(activeBit.tdi ? 1 : 0);
            tckOut.setValue(clockIn.getBool() ? 1 : 0);
        } else {
            tmsOut.setValue(0);
            tdiOut.setValue(0);
            tckOut.setValue(0);
        }
    }

    // -------------------------------------------------------------------------
    // GDB server thread
    // -------------------------------------------------------------------------

    private void runGdbServer() {
        ServerSocket server = bindServer();
        if (server == null) {
            System.err.println("GDB: could not bind to any port in [" + BASE_GDB_PORT + ", " + MAX_GDB_PORT + "]");
            return;
        }
        serverSocket = server;
        System.out.println("GDB: listening on port " + server.getLocalPort());

        try {
            while (!Thread.currentThread().isInterrupted()) {
                Socket client;
                try {
                    client = server.accept();
                } catch (SocketException e) {
                    break;
                } catch (IOException e) {
                    continue;
                }

                try {
                    client.setTcpNoDelay(true);
                    System.out.println("GDB: client connected from " + client.getRemoteSocketAddress());
                    activeClient = client;
                    serveClient(client);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
                } catch (Exception e) {
                    System.err.println("GDB: client error: " + e.getMessage());
                } finally {
                    activeClient = null;
                    try {
                        client.close();
                    } catch (IOException ignored) {

                    }
                    System.out.println("GDB: client disconnected");
                }
            }
        } finally {
            try {
                server.close();
            } catch (IOException ignored) {

            }
            serverSocket = null;
        }
    }

    private static ServerSocket bindServer() {
        for (int port = BASE_GDB_PORT; port <= MAX_GDB_PORT; port++) {
            try {
                ServerSocket ss = new ServerSocket();
                ss.setReuseAddress(true);
                ss.bind(new InetSocketAddress(port));
                return ss;
            } catch (IOException ignored) {

            }
        }
        return null;
    }

    private void serveClient(Socket client) throws IOException, InterruptedException {
        InputStream  in  = client.getInputStream();
        OutputStream out = client.getOutputStream();

        System.out.println("TAP reset");
        tapReset();

        // Ensure DM is ready: clear any leftover cmderr.
        // The RISC-V spec mandates that progbuf writes are SILENTLY IGNORED if cmderr != 0.
        System.out.println("Read abstractcs");
        long abstractcs = dmiRead(DMI_ABSTRACTCS);
        if (((abstractcs >> 8) & 0x7L) != 0) {
            System.out.println("GDB: Clearing residual cmderr on new connection.");
            dmiWrite(DMI_ABSTRACTCS, 0x7L << 8);
        }

        haltCore();

        while (!Thread.currentThread().isInterrupted()) {
            RspPacket pkt = readPacket(in);
            if (pkt == null) break;

            if (pkt.valid) {
                out.write('+');
                out.flush();
            } else {
                out.write('-');
                out.flush();
                continue;
            }

            if (pkt.payload.equals("c")) {
                resumeCore();

                while (true) {
                    long status = dmiRead(DMI_DMSTATUS);

                    if ((status & DMSTATUS_ALLHALTED) != 0) {
                        sendPacket(out, "S05");
                        break;
                    }

                    if (in.available() > 0) {
                        int b = in.read();
                        if (b == 0x03) {
                            haltCore();
                            sendPacket(out, "S02");
                            break;
                        }
                    }
                }
                continue; // Response already sent; skip processGdbCommand.
            }

            String response = processGdbCommand(pkt.payload);
            sendPacket(out, response);
        }
    }

    private String processGdbCommand(String cmd) throws InterruptedException {
        if (cmd.startsWith("qSupported")) return "PacketSize=4000;qXfer:features:read+";
        if (cmd.startsWith("qXfer:features:read:target.xml:")) {
            String[] parts = cmd.substring("qXfer:features:read:target.xml:".length()).split(",");
            int offset = Integer.parseInt(parts[0], 16);
            int length = Integer.parseInt(parts[1], 16);
            return qXferSlice(TARGET_XML, offset, length);
        }
        if (cmd.equals("?")) return "S05";
        if (cmd.equals("g")) return readAllRegisters();
        if (cmd.startsWith("G")) {
            writeAllRegisters(cmd.substring(1));
            return "OK";
        }
        if (cmd.startsWith("p")) return readRegister(Integer.parseInt(cmd.substring(1), 16));
        if (cmd.startsWith("P")) {
            int eq = cmd.indexOf('=');
            writeRegister(Integer.parseInt(cmd.substring(1, eq), 16), (int) Long.parseUnsignedLong(cmd.substring(eq + 1), 16));
            return "OK";
        }
        if (cmd.startsWith("m")) {
            System.out.println("Received m");
            String[] parts = cmd.substring(1).split(",");
            return readMemory(Long.parseUnsignedLong(parts[0], 16), Integer.parseInt(parts[1], 16));
        }
        if (cmd.startsWith("M")) {
            System.out.println("Received M");
            int colon = cmd.indexOf(':');
            writeMemory(Long.parseUnsignedLong(cmd.substring(1, colon).split(",")[0], 16), cmd.substring(colon + 1));
            return "OK";
        }
        if (cmd.equals("s")) {
            singleStep();
            return "S05";
        }
        return "";
    }

    private static String qXferSlice(String full, int offset, int length) {
        if (offset >= full.length()) return "l";
        String chunk = full.substring(offset, Math.min(offset + length, full.length()));
        return ((offset + length) >= full.length() ? "l" : "m") + chunk;
    }

    private static RspPacket readPacket(InputStream in) throws IOException {
        int c;
        do {
            c = in.read();
            if (c == -1) return null;
        } while (c != '$');

        StringBuilder sb = new StringBuilder();
        int sum = 0;
        while (true) {
            c = in.read();
            if (c == -1) return null;
            if (c == '#') break;
            sb.append((char) c);
            sum = (sum + c) & 0xFF;
        }

        int hi = in.read();
        int lo = in.read();
        if (hi == -1 || lo == -1) return null;
        boolean valid = (Integer.parseInt(String.valueOf((char) hi) + (char) lo, 16) == sum);

        return new RspPacket(sb.toString(), valid);
    }

    private static void sendPacket(OutputStream out, String payload) throws IOException {
        byte[] bytes = payload.getBytes("US-ASCII");
        int sum = 0;
        for (byte b : bytes) sum = (sum + (b & 0xFF)) & 0xFF;

        out.write('$');
        out.write(bytes);
        out.write('#');
        out.write(String.format("%02x", sum).getBytes("US-ASCII"));
        out.flush();
    }

    private static final class RspPacket {
        final String  payload;
        final boolean valid;

        RspPacket(String payload, boolean valid) {
            this.payload = payload;
            this.valid   = valid;
        }
    }

    // -------------------------------------------------------------------------
    // Register / memory operations — all via progbuf
    // -------------------------------------------------------------------------

    private static final int GDB_REG_PC  = 32;
    private static final int GDB_REG_DPC = 6065;

    private String readRegister(int reg) throws InterruptedException {
        return toLittleEndianHex((int) progbufReadReg(reg));
    }

    private String readAllRegisters() throws InterruptedException {
        System.out.println("Reading all registers...");
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < 33; i++) {
            sb.append(readRegister(i));
        }
        return sb.toString();
    }

    private void writeRegister(int reg, int value) throws InterruptedException {
        progbufWriteReg(reg, value & 0xFFFFFFFFL);
    }

    private void writeAllRegisters(String hexData) throws InterruptedException {
        for (int i = 0; i < 33; i++) {
            writeRegister(i, (int) fromLittleEndianHex(hexData.substring(i * 8, i * 8 + 8)));
        }
    }

    private String readMemory(long addr, int len) throws InterruptedException {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < len; i++) {
            sb.append(String.format("%02x", progbufReadByte(addr + i) & 0xFFL));
        }
        return sb.toString();
    }

    private void writeMemory(long addr, String hexData) throws InterruptedException {
        for (int i = 0; i < hexData.length() / 2; i++) {
            progbufWriteByte(addr + i, Long.parseUnsignedLong(hexData.substring(i * 2, i * 2 + 2), 16));
        }
    }

    // -------------------------------------------------------------------------
    // Progbuf primitives
    // -------------------------------------------------------------------------

    private void executeProgbuf() throws InterruptedException {
        System.out.println("executeProgbuf");
        dmiWrite(DMI_COMMAND, CMD_POSTEXEC);
        waitForAbstractDone();
    }

    private long progbufReadReg(int reg) throws InterruptedException {
        System.out.println("progbufReadReg");
        dmiWrite(DMI_PROGBUF0, INSN_CSRW_DSCRATCH0_X10);
        dmiWrite(DMI_PROGBUF1, INSN_CSRW_DSCRATCH1_X11);
        dmiWrite(DMI_PROGBUF2, INSN_LUI_X11_1);
        System.out.println("Read Register: " + reg);
        if (reg == GDB_REG_PC || reg == GDB_REG_DPC) {
            dmiWrite(DMI_PROGBUF3, INSN_CSRR_X10_DPC);
            dmiWrite(DMI_PROGBUF4, INSN_SW_BASE | (10L << 20));
            dmiWrite(DMI_PROGBUF5, INSN_CSRR_X10_DSCRATCH0);
            dmiWrite(DMI_PROGBUF6, INSN_CSRR_X11_DSCRATCH1);
            dmiWrite(DMI_PROGBUF7, INSN_EBREAK);
        } else {
            dmiWrite(DMI_PROGBUF3, INSN_SW_BASE | ((long) reg << 20));
            dmiWrite(DMI_PROGBUF4, INSN_CSRR_X10_DSCRATCH0);
            dmiWrite(DMI_PROGBUF5, INSN_CSRR_X11_DSCRATCH1);
            dmiWrite(DMI_PROGBUF6, INSN_EBREAK);
        }

        executeProgbuf();
        System.out.println("Now reading out from data0 for read reg");
        return dmiRead(DMI_DATA0);
    }

    private void progbufWriteReg(int reg, long value) throws InterruptedException {
        dmiWrite(DMI_DATA0, value & 0xFFFFFFFFL);
        dmiWrite(DMI_PROGBUF0, INSN_CSRW_DSCRATCH0_X10);
        dmiWrite(DMI_PROGBUF1, INSN_CSRW_DSCRATCH1_X11);
        dmiWrite(DMI_PROGBUF2, INSN_LUI_X11_1);

        if (reg == GDB_REG_PC || reg == GDB_REG_DPC) {
            dmiWrite(DMI_PROGBUF3, INSN_LW_BASE | (10L << 7));
            dmiWrite(DMI_PROGBUF4, INSN_CSRW_DPC_X10);
            dmiWrite(DMI_PROGBUF5, INSN_CSRR_X10_DSCRATCH0);
            dmiWrite(DMI_PROGBUF6, INSN_CSRR_X11_DSCRATCH1);
            dmiWrite(DMI_PROGBUF7, INSN_EBREAK);
        } else if (reg == 10) {
            dmiWrite(DMI_PROGBUF3, INSN_LW_BASE | (10L << 7));
            dmiWrite(DMI_PROGBUF4, INSN_CSRR_X11_DSCRATCH1);
            dmiWrite(DMI_PROGBUF5, INSN_EBREAK);
        } else if (reg == 11) {
            dmiWrite(DMI_PROGBUF3, INSN_LW_BASE | ((long) reg << 7));
            dmiWrite(DMI_PROGBUF4, INSN_CSRR_X10_DSCRATCH0);
            dmiWrite(DMI_PROGBUF5, INSN_EBREAK);
        } else {
            dmiWrite(DMI_PROGBUF3, INSN_LW_BASE | ((long) reg << 7));
            dmiWrite(DMI_PROGBUF4, INSN_CSRR_X10_DSCRATCH0);
            dmiWrite(DMI_PROGBUF5, INSN_CSRR_X11_DSCRATCH1);
            dmiWrite(DMI_PROGBUF6, INSN_EBREAK);
        }

        executeProgbuf();
    }

    private long progbufReadByte(long addr) throws InterruptedException {
        long hi  = (addr + 0x800L) >> 12;
        long lo  = addr - (hi << 12);
        long insnAddiX11 = ((lo & 0xFFFL) << 20) | (11L << 15) | (0L << 12) | (11L << 7) | 0x13L;
        long insnLuiX11  = ((hi & 0xFFFFFL) << 12) | (11L << 7) | 0x37L;

        dmiWrite(DMI_PROGBUF0, INSN_CSRW_DSCRATCH0_X10);
        dmiWrite(DMI_PROGBUF1, INSN_CSRW_DSCRATCH1_X11);
        dmiWrite(DMI_PROGBUF2, insnLuiX11);
        dmiWrite(DMI_PROGBUF3, insnAddiX11);
        dmiWrite(DMI_PROGBUF4, INSN_LBU_X10_X11);
        dmiWrite(DMI_PROGBUF5, INSN_LUI_X11_1);
        dmiWrite(DMI_PROGBUF6, INSN_SW_BASE | (10L << 20));
        dmiWrite(DMI_PROGBUF7, INSN_EBREAK);

        executeProgbuf();
        return dmiRead(DMI_DATA0);
    }

    private void progbufWriteByte(long addr, long byteVal) throws InterruptedException {
        long hi  = (addr + 0x800L) >> 12;
        long lo  = addr - (hi << 12);
        long insnAddiX11 = ((lo & 0xFFFL) << 20) | (11L << 15) | (0L << 12) | (11L << 7) | 0x13L;
        long insnLuiX11  = ((hi & 0xFFFFFL) << 12) | (11L << 7) | 0x37L;

        dmiWrite(DMI_DATA0,    byteVal & 0xFFL);
        dmiWrite(DMI_PROGBUF0, INSN_CSRW_DSCRATCH0_X10);
        dmiWrite(DMI_PROGBUF1, INSN_CSRW_DSCRATCH1_X11);
        dmiWrite(DMI_PROGBUF2, INSN_LUI_X11_1);
        dmiWrite(DMI_PROGBUF3, INSN_LW_BASE | (10L << 7));
        dmiWrite(DMI_PROGBUF4, insnLuiX11);
        dmiWrite(DMI_PROGBUF5, insnAddiX11);
        dmiWrite(DMI_PROGBUF6, INSN_SB_X10_X11);
        dmiWrite(DMI_PROGBUF7, INSN_EBREAK);

        executeProgbuf();
    }

    private long progbufReadDcsr() throws InterruptedException {
        dmiWrite(DMI_PROGBUF0, INSN_CSRW_DSCRATCH0_X10);
        dmiWrite(DMI_PROGBUF1, INSN_CSRW_DSCRATCH1_X11);
        dmiWrite(DMI_PROGBUF2, INSN_LUI_X11_1);
        dmiWrite(DMI_PROGBUF3, INSN_CSRR_X10_DCSR);
        dmiWrite(DMI_PROGBUF4, INSN_SW_BASE | (10L << 20));
        dmiWrite(DMI_PROGBUF5, INSN_CSRR_X10_DSCRATCH0);
        dmiWrite(DMI_PROGBUF6, INSN_CSRR_X11_DSCRATCH1);
        dmiWrite(DMI_PROGBUF7, INSN_EBREAK);

        executeProgbuf();
        return dmiRead(DMI_DATA0);
    }

    private void progbufWriteDcsr(long value) throws InterruptedException {
        System.out.println("progbufWriteDcsr DMI writes");
        System.out.println("Value is: " + value);
        dmiWrite(DMI_DATA0, value & 0xFFFFFFFFL);
        dmiWrite(DMI_PROGBUF0, INSN_CSRW_DSCRATCH0_X10);
        dmiWrite(DMI_PROGBUF1, INSN_CSRW_DSCRATCH1_X11);
        dmiWrite(DMI_PROGBUF2, INSN_LUI_X11_1);
        dmiWrite(DMI_PROGBUF3, INSN_LW_BASE | (10L << 7));
        dmiWrite(DMI_PROGBUF4, INSN_CSRW_DCSR_X10);
        dmiWrite(DMI_PROGBUF5, INSN_CSRR_X10_DSCRATCH0);
        dmiWrite(DMI_PROGBUF6, INSN_CSRR_X11_DSCRATCH1);
        dmiWrite(DMI_PROGBUF7, INSN_EBREAK);

        System.out.println("progbufWriteDcsr execute progbuf");
        executeProgbuf();
    }

    // -------------------------------------------------------------------------
    // Core run-control
    // -------------------------------------------------------------------------

    private void resumeCore() throws InterruptedException {
        dmiWrite(DMI_DMCONTROL, DMCONTROL_DMACTIVE | DMCONTROL_RESUMEREQ);
        dmiWrite(DMI_DMCONTROL, DMCONTROL_DMACTIVE);
    }

    private void waitForHalt() throws InterruptedException {
        System.out.println("WAITING FOR HALT...");
        for (;;) {
            long status = dmiRead(DMI_DMSTATUS);
            if ((status & DMSTATUS_ALLHALTED) != 0) break;
        }
        System.out.println("HALTED.");
    }

    private void haltCore() throws InterruptedException {
        dmiWrite(DMI_DMCONTROL, DMCONTROL_DMACTIVE | DMCONTROL_HALT_REQ);
        waitForHalt();
        dmiWrite(DMI_DMCONTROL, DMCONTROL_DMACTIVE);
    }

    private void waitForAbstractDone() throws InterruptedException {
        System.out.println("waitForAbstractDone");
        long abstractcs;
        do {
            abstractcs = dmiRead(DMI_ABSTRACTCS);
        } while ((abstractcs & (1L << 12)) != 0);

        int cmderr = (int) ((abstractcs >> 8) & 0x7L);
        if (cmderr != 0) {
            System.err.println("GDB: abstractcs cmderr=" + cmderr + " -- clearing");
            dmiWrite(DMI_ABSTRACTCS, 0x7L << 8);
        }
    }

    private void singleStep() throws InterruptedException {
        System.out.println("Reading DCSR");
        long dcsr = progbufReadDcsr();
        System.out.println("Writing DCSR for step");
        progbufWriteDcsr(dcsr | (1L << 2));
        System.out.println("Resuming core");
        resumeCore();
        System.out.println("Waiting for halt");
        waitForHalt();
        progbufWriteDcsr(dcsr & ~(1L << 2));
    }

    // -------------------------------------------------------------------------
    // JTAG DMI layer
    // -------------------------------------------------------------------------

    /**
     * Blocks until the DMI reports idle (op == SUCCESS) by shifting NOPs.
     * Must be called with IR already pointing at DMI.
     * A NOP that returns BUSY does not latch a sticky error — safe to poll
     * indefinitely. Any other non-SUCCESS status is logged and we bail out.
     */
    private void dmiDrainBusy() throws InterruptedException {
        while (true) {
            long result = jtagShift(false, (long) DMI_OP_NOP, DMI_TOTAL_BITS);
            int op = (int) (result & 0x3L);
            if (op == DMI_STATUS_SUCCESS) return;
            if (op != DMI_STATUS_BUSY) {
                System.err.println("GDB: unexpected op=" + op + " while draining DMI");
                return;
            }
            // op == BUSY: target still processing; keep polling.
        }
    }

    /**
     * Resets the sticky-busy latch in DTMCS and waits until the DMI is truly
     * idle before returning. After this call the IR is left pointing at DMI.
     */
    private void clearStickyError() throws InterruptedException {
        jtagShift(true, IR_DTMCS, IR_BITS);
        jtagShift(false, DTMCS_DMIRESET, 32);
        // Re-select DMI and drain: the target may still be processing whatever
        // caused the collision; we must wait for idle before the caller retries.
        jtagShift(true, IR_DMI, IR_BITS);
        dmiDrainBusy();
    }

    /**
     * Performs a single DMI write and blocks until the target has finished
     * processing it. Never returns while the DMI is still busy.
     */
    private void dmiWrite(long addr, long data) throws InterruptedException {
        System.out.println("dmiWrite");
        long dmiWord = (addr << 34) | ((data & 0xFFFFFFFFL) << 2) | DMI_OP_WRITE;
        System.out.println("addr is: " + addr + " and data is: " + data);
        System.out.println("dmiWord is: " + dmiWord);

        while (true) {
            // Always select IR_DMI at the top of the retry loop so that after a
            // clearStickyError (which leaves IR at DMI but costs many cycles) we
            // are still correctly positioned.
            jtagShift(true, IR_DMI, IR_BITS);
            long result = jtagShift(false, dmiWord, DMI_TOTAL_BITS);
            int status = (int) (result & 0x3L);

            if (status == DMI_STATUS_BUSY) {
                // A real write collided — sticky latch is now set.  Clear it,
                // drain until idle, then retry the write.
                System.out.println("Sticky error while writing, retrying..");
                clearStickyError();
                continue;
            }
            if (status != DMI_STATUS_SUCCESS) {
                System.err.println("GDB: DMI write error for addr 0x" + Long.toHexString(addr));
                return;
            }

            // Write was accepted by the DMI.  Now drain until the target has
            // finished acting on it so the next operation cannot collide.
            dmiDrainBusy();
            return;
        }
    }

    /**
     * Performs a single DMI read and returns the 32-bit result.
     * Blocks until the target has finished and the data is ready.
     */
    private long dmiRead(long addr) throws InterruptedException {
        long dmiWord = (addr << 34) | DMI_OP_READ;

        // Phase 1: issue the read request.  Retry if the initiation itself
        // collides (target was still busy from the previous operation).
        while (true) {
            jtagShift(true, IR_DMI, IR_BITS);
            long result = jtagShift(false, dmiWord, DMI_TOTAL_BITS);
            int status = (int) (result & 0x3L);

            if (status == DMI_STATUS_SUCCESS) break;
            if (status == DMI_STATUS_BUSY) {
                clearStickyError();
                continue;
            }
            System.err.println("GDB: DMI read initiation error for addr 0x" + Long.toHexString(addr));
            return 0;
        }

        // Phase 2: poll NOPs until the read data is ready.
        // IR is still pointing at DMI from the last jtagShift above.
        while (true) {
            long result = jtagShift(false, (long) DMI_OP_NOP, DMI_TOTAL_BITS);
            int op = (int) (result & 0x3L);

            if (op == DMI_STATUS_SUCCESS) return (result >> 2) & 0xFFFFFFFFL;
            if (op == DMI_STATUS_BUSY)    continue; // not a sticky error; keep polling
            System.err.println("GDB: DMI read poll error for addr 0x" + Long.toHexString(addr));
            return 0;
        }
    }

    // -------------------------------------------------------------------------
    // JTAG TAP layer
    // -------------------------------------------------------------------------

    private void tapReset() throws InterruptedException {
        for (int i = 0; i < 5; i++) shiftBit(true,  false);
        shiftBit(false, false);
    }

    private long jtagShift(boolean selectIR, long data, int nbits) throws InterruptedException {
        shiftBit(true,  false);
        if (selectIR) shiftBit(true, false);
        shiftBit(false, false);
        shiftBit(false, false);

        long tdoBits = 0L;
        for (int i = 0; i < nbits; i++) {
            if (shiftBit((i == nbits - 1), ((data >> i) & 1L) == 1L)) {
                tdoBits |= (1L << i);
            }
        }

        shiftBit(true,  false);
        shiftBit(false, false);
        return tdoBits;
    }

    private boolean shiftBit(boolean tms, boolean tdi) throws InterruptedException {
        BitRequest req = new BitRequest(tms, tdi);
        pendingBit.put(req);
        return completedBit.take().getTdo();
    }

    // -------------------------------------------------------------------------
    // Data-format helpers
    // -------------------------------------------------------------------------

    private static String toLittleEndianHex(int v) {
        return String.format("%02x%02x%02x%02x",
                v & 0xFF, (v >> 8) & 0xFF, (v >> 16) & 0xFF, (v >> 24) & 0xFF);
    }

    private static long fromLittleEndianHex(String hex) {
        long b0 = Long.parseUnsignedLong(hex.substring(0, 2), 16);
        long b1 = Long.parseUnsignedLong(hex.substring(2, 4), 16);
        long b2 = Long.parseUnsignedLong(hex.substring(4, 6), 16);
        long b3 = Long.parseUnsignedLong(hex.substring(6, 8), 16);
        return b0 | (b1 << 8) | (b2 << 16) | (b3 << 24);
    }

    static final class BitRequest {
        final boolean tms;
        final boolean tdi;
        private boolean tdo;

        BitRequest(boolean tms, boolean tdi) {
            this.tms = tms;
            this.tdi = tdi;
        }

        void setTdo(boolean value) {
            tdo = value;
        }
        boolean getTdo() {
            return tdo;
        }
    }
}
