const std = @import("std");
const fs = std.fs;
const mem = @import("std").mem;
const assert = std.debug.assert;
const expect = std.testing.expect;

// Simulator: http://spacetech.github.io/LC3Simulator/
// https://wchargin.com/lc3web/
// Specification: https://justinmeiners.github.io/lc3-vm/supplies/lc3-isa.pdf

const memory_size = 65536;
const program_start_offset = 0x3000;

const Reg = enum(u16) { R0 = 0, R1, R2, R3, R4, R5, R6, R7, PC, COND, COUNT };

const Op = enum(u16) {
// branch
BR = 0,
// add
ADD,
// load
LD,
// store
ST,
// jump register
JSR,
// bitwise and
AND,
// load register
LDR,
// store register
STR,
// unused
RTI,
// bitwise not
NOT,
// load indirect
LDI,
// store indirect
STI,
// jump
JMP,
// reserved (unused)
RES,
// load effective address
LEA,
// execute trap
TRAP };

const Flag = enum(u8) {
    POS = 1 << 0,
    ZRO = 1 << 1,
    NEG = 1 << 2,
};

fn sign_extend(val: u16, comptime bit_count: u16) u16 {
    var extended: u16 = val;

    // When negative sign, extend with 1's to maintain "negative" values.
    if (extended & (1 << bit_count - 1) > 0) {
        extended |= @truncate(u16, (0xFFFF << bit_count));
        return extended;
    }

    return extended;
}

const LC3 = struct {
    running: bool = false,
    memory: [memory_size]u16 = undefined,
    reg: [@enumToInt(Reg.COUNT)]u16 = undefined,

    pub fn init() LC3 {
        var lc3 = LC3{};
        lc3.reset();
        return lc3;
    }

    pub fn reset(self: *LC3) void {
        self.running = false;

        self.memory = [_]u16{0} ** memory_size;
        self.reg = [_]u16{0} ** @enumToInt(Reg.COUNT);

        self.reg[@enumToInt(Reg.PC)] = program_start_offset;
        self.reg[@enumToInt(Reg.COND)] = @enumToInt(Flag.ZRO);
    }

    pub fn load_rom(self: *LC3) !void {
        var f = try fs.cwd().openFile("src/roms/rogue.obj", .{});
        defer f.close();

        var buf_reader = std.io.bufferedReader(f.reader());
        var in_stream = buf_reader.reader();

        var buf: [1024]u8 = undefined;
        var offset: usize = 0;

        while (try in_stream.readUntilDelimiterOrEof(&buf, '\n')) |line| {
            for (line) |val| {
                self.memory[program_start_offset + offset] = val;
                offset += 1;
            }
        }
    }

    pub fn update_flags(self: *LC3, r: u16) void {
        if (self.reg[r] == 0) {
            self.reg[@enumToInt(Reg.COND)] = @enumToInt(Flag.ZRO);
        } else if ((self.reg[r] >> 15) > 0) {
            self.reg[@enumToInt(Reg.COND)] = @enumToInt(Flag.NEG);
        } else {
            self.reg[@enumToInt(Reg.COND)] = @enumToInt(Flag.POS);
        }
    }

    pub fn dump_registers(self: *LC3) void {
        std.debug.warn("\n", .{});
        var i: usize = 0;
        while (i < self.reg.len) : (i += 1) {
            std.debug.warn("  reg[{d}] => {d}/{d}/({x})\n", .{ i, self.reg[i], @bitCast(i16, self.reg[i]), self.reg[i] });
        }
    }

    pub fn start(self: *LC3) void {
        self.running = true;

        while (self.running) {
            const instr = self.memory[self.reg[@enumToInt(Reg.PC)]];
            self.reg[@enumToInt(Reg.PC)] = self.reg[@enumToInt(Reg.PC)] + 1;

            const op = @intToEnum(Op, instr >> 12);
            switch (op) {
                .ADD => self.op_add(instr),
                .LD => self.op_ld(instr),
                .JMP => self.op_jmp(instr),
                .AND => self.op_and(instr),
                .NOT => self.op_not(instr),
                .LDI => self.op_ldi(instr),

                .RES => self.op_ignore(instr),
                .RTI => self.op_ignore(instr),
                .BR => self.op_branch(instr),

                // TODO ops.
                .ST => self.op_nop(instr),
                .JSR => self.op_nop(instr),
                .LDR => self.op_nop(instr),
                .STR => self.op_nop(instr),
                .STI => self.op_nop(instr),
                .LEA => self.op_nop(instr),
                .TRAP => self.op_nop(instr),
            }
        }
    }

    pub fn op_ignore(_: *LC3, _: u16) void {
        @panic("instruction not implemented by design!");
    }

    pub fn op_nop(_: *LC3, _: u16) void {}

    pub fn op_not(self: *LC3, instr: u16) void {
        const dr = (instr >> 9) & 0x7;
        const lr = (instr >> 6) & 0x7;

        self.reg[dr] = ~self.reg[lr];

        self.update_flags(dr);
    }

    pub fn op_add(self: *LC3, instr: u16) void {
        const dr = (instr >> 9) & 0x7;
        const lr = (instr >> 6) & 0x7;

        const imm_mode = (instr >> 5) & 0x1;

        if (imm_mode > 0) {
            const imm5 = sign_extend(instr & 0x1f, 5);
            self.reg[dr] = self.reg[lr] + imm5;
        } else {
            const rr = instr & 0x7;
            self.reg[dr] = self.reg[lr] + self.reg[rr];
        }

        self.update_flags(dr);
    }

    pub fn op_and(self: *LC3, instr: u16) void {
        const dr = (instr >> 9) & 0x7;
        const lr = (instr >> 6) & 0x7;

        const imm_mode = (instr >> 5) & 0x1;

        if (imm_mode > 0) {
            const imm5 = sign_extend(instr & 0x1f, 5);
            self.reg[dr] = self.reg[lr] & imm5;
        } else {
            const rr = instr & 0x7;
            self.reg[dr] = self.reg[lr] & self.reg[rr];
        }

        self.update_flags(dr);
    }

    pub fn op_jmp(self: *LC3, instr: u16) void {
        // per the docs, JMP is also a stand-in for RET via r7 done by assemblers.
        const jr = self.reg[(instr >> 6) & 0x7];
        self.reg[@enumToInt(Reg.PC)] = jr;
    }

    pub fn op_ld(self: *LC3, instr: u16) void {
        const dr = (instr >> 9) & 0x7;

        const pc_offset = sign_extend(instr & 0x1ff, 9);
        self.reg[dr] = self.memory[self.reg[@enumToInt(Reg.PC)] + pc_offset];

        self.update_flags(dr);
    }

    pub fn op_ldi(self: *LC3, instr: u16) void {
        const dr = (instr >> 9) & 0x7;

        const pc_offset = sign_extend(instr & 0x1ff, 9);
        self.reg[dr] = self.memory[self.memory[self.reg[@enumToInt(Reg.PC)] + pc_offset]];

        self.update_flags(dr);
    }

    pub fn op_branch(self: *LC3, instr: u16) void {
        const cond_flag = (instr >> 9) & 0x7;

        if (cond_flag & self.reg[@enumToInt(Reg.PC)]) {
            const pc_offset = sign_extend(instr & 0x1ff, 9);
            self.reg[@enumToInt(Reg.PC)] = self.reg[@enumToInt(Reg.PC)] + pc_offset;
        }
    }
};

pub fn main() anyerror!void {
    std.log.info("All your codebase are belong to us.", .{});

    var lc3 = LC3.init();
    lc3.reset();

    try lc3.load_rom();

    //std.log.info("dump: {s}", .{lc3});
    //lc3.start();
    var x: u8 = 0xfb;
    var y: u16 = x;
    std.log.info("before: {b}", .{y});
    const result = sign_extend(y, 5);
    std.log.info("after : {b}", .{result});
}

test "op_add" {
    var lc3 = LC3.init();

    // AND	R0 R0 #0 (clear to zero)
    lc3.op_and(0x5020);
    // ADD R0, R0, #1 (add immediate 1)
    lc3.op_add(0x1021);
    // ADD R0, R0, #6 (add immediate 6)
    lc3.op_add(0x1026);

    try expect(lc3.reg[0x0] == 0x7);

    //TODO: assert register and flag state.
}

test "op_not" {
    var lc3 = LC3.init();

    lc3.op_and(0x5020);
    lc3.op_add(0x1026);
    lc3.op_not(0x903F);

    try expect(lc3.reg[0x0] == 0xFFF9);
    //TODO: assert register and flag state.
}
