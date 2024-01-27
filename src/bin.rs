use std::io::Read;

mod w65c02;

const _BACKTRACE_ENABLE:bool = true;

fn main() {
    if _BACKTRACE_ENABLE {
        std::env::set_var("RUST_BACKTRACE", "1");
    }
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 2 {
        println!("Usage: {} <filename>", args[0]);
        return;
    }
    let memory_bin_filename = &args[1];
    let mut memory = [0; 0x10000];
    let mut memory_bin_file = std::fs::File::open(memory_bin_filename)
        .expect(format!("Failed to open memory binary file.\n{}", memory_bin_filename).as_str());
    let mut buf = Vec::new();
    memory_bin_file.read_to_end(&mut buf).unwrap();
    if buf.len() != 0x10000 {
        println!("Memory binary file must be exactly 0x10000 bytes long., but it is {:#04x} bytes long.", buf.len());
        return;
    }
    for i in 0..0x10000 {
        memory[i] = buf[i];
    }
    let mut processor = w65c02::Processor::new(memory);
    let mut s = String::new();
    loop {
        processor.tick(true);
        _ = std::io::stdin().read_line(&mut s);
    }
}
