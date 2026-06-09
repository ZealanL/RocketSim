use log::LevelFilter;

pub fn try_init() -> Result<(), log::SetLoggerError> {
    fern::Dispatch::new()
        .format(|out, message, record| {
            out.finish(format_args!("[RSIM | {}] {}", record.level(), message));
        })
        .level(LevelFilter::Info)
        .chain(std::io::stderr())
        .apply()
}
