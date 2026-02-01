//! Slicer CLI - Command-line interface for the slicer library
//!
//! Usage:
//!   slicer-cli slice <input.stl> -o <output.gcode> [options]
//!   slicer-cli slice <input.stl> --printer bambu-lab-h2d --filament bambu-pla-basic
//!   slicer-cli slice <input.stl> --config my_config.json
//!   slicer-cli validate <input.stl> <reference.gcode> [options]
//!   slicer-cli info <input.stl>
//!   slicer-cli list-printers
//!   slicer-cli list-filaments

use anyhow::{Context, Result};
use clap::{Parser, Subcommand};
use indicatif::{ProgressBar, ProgressStyle};
use log::{info, warn, LevelFilter};
use slicer::config::InfillPattern;
use slicer::gcode::compare::ParsedGCode;
use slicer::gcode::validation::{ReportFormat, ValidationConfig, ValidationReport};
use slicer::mesh::load_stl;
use slicer::pipeline::{PipelineConfig, PrintPipeline};
use slicer::profiles::{ProfileRegistry, SliceConfig};
use slicer::support::SupportPattern;
use std::fs;
use std::path::PathBuf;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;

/// A Rust rewrite of the BambuStudio core slicing algorithm
#[derive(Parser, Debug)]
#[command(name = "slicer-cli")]
#[command(author, version, about, long_about = None)]
struct Cli {
    /// Enable verbose output
    #[arg(short, long, global = true)]
    verbose: bool,

    /// Enable debug output
    #[arg(short, long, global = true)]
    debug: bool,

    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand, Debug)]
enum Commands {
    /// Slice an STL file and generate G-code
    Slice {
        /// Input STL file
        #[arg(value_name = "INPUT")]
        input: PathBuf,

        /// Output G-code file
        #[arg(short, long, value_name = "OUTPUT")]
        output: Option<PathBuf>,

        /// Slice configuration file (JSON format) - overrides --printer and --filament
        #[arg(short, long, value_name = "CONFIG")]
        config: Option<PathBuf>,

        /// Printer profile ID (e.g., 'bambu-lab-h2d')
        #[arg(long, value_name = "PRINTER_ID")]
        printer: Option<String>,

        /// Filament profile ID (e.g., 'bambu-pla-basic')
        #[arg(long, value_name = "FILAMENT_ID")]
        filament: Option<String>,

        /// Nozzle diameter in mm (defaults to printer's default)
        #[arg(long)]
        nozzle: Option<f64>,

        /// Data directory containing profiles (default: 'data' relative to executable)
        #[arg(long, value_name = "DIR")]
        data_dir: Option<PathBuf>,

        /// Layer height in mm
        #[arg(long, default_value = "0.2")]
        layer_height: f64,

        /// First layer height in mm
        #[arg(long, default_value = "0.2")]
        first_layer_height: f64,

        /// Number of perimeters
        #[arg(long, default_value = "3")]
        perimeters: u32,

        /// Infill density (0-100)
        #[arg(long, default_value = "20")]
        infill_density: u32,

        /// Infill pattern
        #[arg(long, default_value = "grid")]
        infill_pattern: String,

        /// Generate supports
        #[arg(long)]
        support: bool,

        /// Support type (normal, tree)
        #[arg(long, default_value = "normal")]
        support_type: String,

        /// Support overhang angle threshold (degrees)
        #[arg(long, default_value = "45")]
        support_angle: f64,

        /// Support density (0-100)
        #[arg(long, default_value = "15")]
        support_density: u32,

        /// Support pattern (grid, lines, honeycomb)
        #[arg(long, default_value = "grid")]
        support_pattern: String,

        /// Only generate supports from build plate
        #[arg(long)]
        support_buildplate_only: bool,

        /// Enable arc fitting (G2/G3 commands) for smoother curves
        #[arg(long)]
        arc_fitting: bool,

        /// Arc fitting tolerance in mm (default: 0.05)
        #[arg(long, default_value = "0.05")]
        arc_tolerance: f64,

        /// Number of threads to use (0 = auto)
        #[arg(short = 'j', long, default_value = "0")]
        threads: usize,
    },

    /// Validate G-code output against a reference file from BambuStudio
    Validate {
        /// Input STL file
        #[arg(value_name = "INPUT")]
        input: PathBuf,

        /// Reference G-code file from BambuStudio
        #[arg(value_name = "REFERENCE")]
        reference: PathBuf,

        /// Configuration file (TOML format)
        #[arg(short, long, value_name = "CONFIG")]
        config: Option<PathBuf>,

        /// Tolerance level: strict, default, or relaxed
        #[arg(long, default_value = "default")]
        tolerance: String,

        /// Output report file
        #[arg(short, long, value_name = "OUTPUT")]
        output: Option<PathBuf>,

        /// Output format: text, json, or html
        #[arg(long, default_value = "text")]
        format: String,

        /// Layer height in mm (should match reference)
        #[arg(long, default_value = "0.2")]
        layer_height: f64,

        /// First layer height in mm
        #[arg(long, default_value = "0.2")]
        first_layer_height: f64,

        /// Number of perimeters
        #[arg(long, default_value = "3")]
        perimeters: u32,

        /// Infill density (0-100)
        #[arg(long, default_value = "15")]
        infill_density: u32,

        /// Pass threshold for quality score (0-100)
        #[arg(long, default_value = "90")]
        pass_threshold: f64,

        /// Only compare without slicing (requires generated G-code path)
        #[arg(long)]
        compare_only: bool,

        /// Path to pre-generated G-code (for --compare-only mode)
        #[arg(long, value_name = "GENERATED")]
        generated: Option<PathBuf>,
    },

    /// Display information about an STL file
    Info {
        /// Input STL file
        #[arg(value_name = "INPUT")]
        input: PathBuf,
    },

    /// Start MCP server for integration with other tools
    #[cfg(feature = "mcp")]
    Serve {
        /// Port to listen on
        #[arg(short, long, default_value = "3000")]
        port: u16,

        /// Host to bind to
        #[arg(long, default_value = "127.0.0.1")]
        host: String,
    },

    /// List available printer profiles
    ListPrinters {
        /// Data directory containing profiles
        #[arg(long, value_name = "DIR")]
        data_dir: Option<PathBuf>,

        /// Filter by brand
        #[arg(long)]
        brand: Option<String>,
    },

    /// List available filament profiles
    ListFilaments {
        /// Data directory containing profiles
        #[arg(long, value_name = "DIR")]
        data_dir: Option<PathBuf>,

        /// Filter by material type
        #[arg(long)]
        material: Option<String>,

        /// Filter by brand
        #[arg(long)]
        brand: Option<String>,
    },
}

fn main() -> Result<()> {
    let cli = Cli::parse();

    // Set up logging
    let log_level = if cli.debug {
        LevelFilter::Debug
    } else if cli.verbose {
        LevelFilter::Info
    } else {
        LevelFilter::Warn
    };

    env_logger::Builder::new()
        .filter_level(log_level)
        .format_timestamp(None)
        .init();

    match cli.command {
        Commands::Slice {
            input,
            output,
            config,
            printer,
            filament,
            nozzle,
            data_dir,
            layer_height,
            first_layer_height,
            perimeters,
            infill_density,
            infill_pattern,
            support,
            support_type,
            support_angle,
            support_density,
            support_pattern,
            support_buildplate_only,
            arc_fitting,
            arc_tolerance,
            threads,
        } => cmd_slice(
            input,
            output,
            config,
            printer,
            filament,
            nozzle,
            data_dir,
            layer_height,
            first_layer_height,
            perimeters,
            infill_density,
            infill_pattern,
            support,
            support_type,
            support_angle,
            support_density,
            support_pattern,
            support_buildplate_only,
            arc_fitting,
            arc_tolerance,
            threads,
        ),
        Commands::Validate {
            input,
            reference,
            config,
            tolerance,
            output,
            format,
            layer_height,
            first_layer_height,
            perimeters,
            infill_density,
            pass_threshold,
            compare_only,
            generated,
        } => cmd_validate(
            input,
            reference,
            config,
            tolerance,
            output,
            format,
            layer_height,
            first_layer_height,
            perimeters,
            infill_density,
            pass_threshold,
            compare_only,
            generated,
        ),
        Commands::Info { input } => cmd_info(input),
        #[cfg(feature = "mcp")]
        Commands::Serve { port, host } => cmd_serve(port, host),
        Commands::ListPrinters { data_dir, brand } => cmd_list_printers(data_dir, brand),
        Commands::ListFilaments {
            data_dir,
            material,
            brand,
        } => cmd_list_filaments(data_dir, material, brand),
    }
}

/// Get the default data directory
fn get_default_data_dir() -> PathBuf {
    // Try relative to executable first
    if let Ok(exe_path) = std::env::current_exe() {
        if let Some(exe_dir) = exe_path.parent() {
            let data_dir = exe_dir.join("data");
            if data_dir.exists() {
                return data_dir;
            }
            // Try parent directory (for development builds)
            if let Some(parent) = exe_dir.parent() {
                let data_dir = parent.join("data");
                if data_dir.exists() {
                    return data_dir;
                }
            }
        }
    }
    // Fall back to current directory
    PathBuf::from("data")
}

#[allow(clippy::too_many_arguments)]
fn cmd_slice(
    input: PathBuf,
    output: Option<PathBuf>,
    config_file: Option<PathBuf>,
    printer_id: Option<String>,
    filament_id: Option<String>,
    nozzle_diameter: Option<f64>,
    data_dir: Option<PathBuf>,
    layer_height: f64,
    first_layer_height: f64,
    perimeters: u32,
    infill_density: u32,
    infill_pattern: String,
    support: bool,
    support_type: String,
    support_angle: f64,
    support_density: u32,
    support_pattern: String,
    support_buildplate_only: bool,
    arc_fitting: bool,
    arc_tolerance: f64,
    threads: usize,
) -> Result<()> {
    info!("Loading STL file: {}", input.display());

    // Determine output path
    let output_path = output.unwrap_or_else(|| input.with_extension("gcode"));

    // Set thread count if specified
    if threads > 0 {
        rayon::ThreadPoolBuilder::new()
            .num_threads(threads)
            .build_global()
            .context("Failed to initialize thread pool")?;
    }

    // Create progress bar
    let progress = ProgressBar::new(100);
    progress.set_style(
        ProgressStyle::default_bar()
            .template("{spinner:.green} [{elapsed_precise}] [{bar:40.cyan/blue}] {pos}% {msg}")
            .unwrap()
            .progress_chars("#>-"),
    );

    progress.set_message("Loading mesh...");
    progress.set_position(5);

    // Load the mesh
    let mesh = load_stl(&input).context("Failed to load STL file")?;

    info!("Mesh loaded:");
    info!("  Triangles: {}", mesh.triangle_count());
    let bb = mesh.compute_bounding_box();
    info!(
        "  Bounding box: ({:.2}, {:.2}, {:.2}) - ({:.2}, {:.2}, {:.2})",
        bb.min.x, bb.min.y, bb.min.z, bb.max.x, bb.max.y, bb.max.z
    );

    progress.set_message("Configuring pipeline...");
    progress.set_position(10);

    // Build pipeline config - either from profiles or CLI arguments
    let config = if let Some(config_path) = config_file {
        // Load from slice config JSON file
        info!("Loading slice config from: {}", config_path.display());
        let slice_config =
            SliceConfig::from_file(&config_path).context("Failed to load slice config file")?;

        let data_path = data_dir.unwrap_or_else(get_default_data_dir);
        info!("Loading profiles from: {}", data_path.display());
        let registry = ProfileRegistry::load_from_directory(&data_path)
            .context("Failed to load profile registry")?;

        info!(
            "  Loaded {} printers, {} filaments",
            registry.printer_count(),
            registry.filament_count()
        );

        let pipeline_config = slice_config
            .build_pipeline_config(&registry)
            .context("Failed to build pipeline config from slice config")?;

        info!("Using slice config:");
        info!("  Printer: {}", slice_config.printer_id);
        info!("  Filament: {}", slice_config.filament_id);
        info!(
            "  Nozzle: {} mm",
            slice_config.nozzle_diameter.unwrap_or(0.4)
        );
        info!("  Layer height: {} mm", pipeline_config.print.layer_height);

        pipeline_config
    } else if let (Some(printer), Some(filament)) = (printer_id, filament_id) {
        // Build config from printer and filament IDs
        info!("Using printer: {}", printer);
        info!("Using filament: {}", filament);

        let data_path = data_dir.unwrap_or_else(get_default_data_dir);
        info!("Loading profiles from: {}", data_path.display());
        let registry = ProfileRegistry::load_from_directory(&data_path)
            .context("Failed to load profile registry")?;

        info!(
            "  Loaded {} printers, {} filaments",
            registry.printer_count(),
            registry.filament_count()
        );

        // Create a slice config from the IDs
        let mut slice_config = SliceConfig::new(&printer, &filament);
        slice_config.nozzle_diameter = nozzle_diameter;

        // Apply CLI overrides to the slice config
        slice_config.quality.layer_height = Some(layer_height);
        slice_config.quality.first_layer_height = Some(first_layer_height);
        slice_config.perimeters.count = Some(perimeters);
        slice_config.infill.density = Some(infill_density as f64);
        slice_config.infill.pattern = Some(infill_pattern.clone());
        slice_config.support.enabled = Some(support);
        if support {
            slice_config.support.support_type = Some(support_type.clone());
            slice_config.support.threshold_angle = Some(support_angle);
            slice_config.support.density = Some(support_density as f64 / 100.0);
            slice_config.support.pattern = Some(support_pattern.clone());
            slice_config.support.buildplate_only = Some(support_buildplate_only);
        }

        let pipeline_config = slice_config
            .build_pipeline_config(&registry)
            .context("Failed to build pipeline config from profiles")?;

        info!("Configuration from profiles:");
        info!(
            "  Nozzle: {} mm",
            slice_config.nozzle_diameter.unwrap_or(0.4)
        );
        info!("  Layer height: {} mm", pipeline_config.print.layer_height);
        info!(
            "  Nozzle temp: {}°C",
            pipeline_config.print.extruder_temperature
        );
        info!("  Bed temp: {}°C", pipeline_config.print.bed_temperature);

        pipeline_config
    } else {
        // Fall back to CLI arguments only (legacy mode)
        info!("Using CLI arguments (no profiles)");

        // Parse infill pattern
        let pattern = match infill_pattern.to_lowercase().as_str() {
            "rectilinear" | "lines" => InfillPattern::Rectilinear,
            "grid" => InfillPattern::Grid,
            "concentric" => InfillPattern::Concentric,
            "honeycomb" => InfillPattern::Honeycomb,
            "gyroid" => InfillPattern::Gyroid,
            _ => {
                warn!("Unknown infill pattern '{}', using grid", infill_pattern);
                InfillPattern::Grid
            }
        };

        info!("Configuration:");
        info!("  Layer height: {} mm", layer_height);
        info!("  First layer height: {} mm", first_layer_height);
        info!("  Perimeters: {}", perimeters);
        info!("  Infill density: {}%", infill_density);
        info!("  Infill pattern: {:?}", pattern);
        info!(
            "  Support: {}",
            if support { "enabled" } else { "disabled" }
        );
        if support {
            info!("  Support type: {}", support_type);
            info!("  Support angle: {}°", support_angle);
            info!("  Support density: {}%", support_density);
            info!("  Support pattern: {}", support_pattern);
            if support_buildplate_only {
                info!("  Support: buildplate only");
            }
        }

        // Configure the pipeline
        let mut config = PipelineConfig::new()
            .layer_height(layer_height)
            .first_layer_height(first_layer_height)
            .perimeters(perimeters)
            .infill_density(infill_density as f64 / 100.0);

        // Enable arc fitting if requested
        if arc_fitting {
            config = config
                .arc_fitting(true)
                .arc_fitting_tolerance(arc_tolerance);
            info!("  Arc fitting: enabled (tolerance: {}mm)", arc_tolerance);
        }

        config.object.fill_pattern = pattern;

        // Configure support if enabled
        if support {
            config = config
                .support_enabled(true)
                .support_overhang_angle(support_angle)
                .support_density(support_density as f64 / 100.0)
                .support_buildplate_only(support_buildplate_only);

            // Parse support pattern
            let sup_pattern = match support_pattern.to_lowercase().as_str() {
                "grid" => SupportPattern::Grid,
                "lines" | "rectilinear" => SupportPattern::Lines,
                "honeycomb" => SupportPattern::Honeycomb,
                "gyroid" => SupportPattern::Gyroid,
                "lightning" => SupportPattern::Lightning,
                _ => {
                    warn!("Unknown support pattern '{}', using grid", support_pattern);
                    SupportPattern::Grid
                }
            };
            config = config.support_pattern(sup_pattern);
        }

        config
    };

    let mut pipeline = PrintPipeline::new(config);

    // Progress tracking for callback
    let progress_value = Arc::new(AtomicU64::new(10));
    let progress_clone = progress.clone();
    let progress_value_clone = progress_value.clone();

    progress.set_message("Slicing...");

    // Run the pipeline with progress callback
    let gcode = pipeline.process_with_callback(&mesh, |stage, stage_progress| {
        let base = match stage {
            "slicing" => 10,
            "support" => 25,
            "processing" => 35,
            "gcode" => 80,
            _ => 10,
        };
        let range = match stage {
            "slicing" => 15,
            "support" => 10,
            "processing" => 45,
            "gcode" => 15,
            _ => 10,
        };
        let pos = base + (stage_progress * range as f64) as u64;
        progress_value_clone.store(pos, Ordering::Relaxed);
        progress_clone.set_position(pos);
        progress_clone.set_message(match stage {
            "slicing" => "Slicing layers...",
            "support" => "Generating supports...",
            "processing" => "Generating toolpaths...",
            "gcode" => "Generating G-code...",
            _ => "Processing...",
        });
    })?;

    progress.set_message("Writing output...");
    progress.set_position(95);

    // Write output
    gcode
        .write_to_file(&output_path)
        .context("Failed to write G-code file")?;

    progress.set_position(100);
    progress.finish_with_message("Done!");

    println!();
    println!("Slicing complete!");
    println!("  Output: {}", output_path.display());
    println!("  Layers: {}", gcode.stats.layer_count);
    println!(
        "  Filament used: {:.2} m",
        gcode.stats.filament_used_mm / 1000.0
    );
    println!("  G-code lines: {}", gcode.line_count());

    Ok(())
}

fn cmd_validate(
    input: PathBuf,
    reference: PathBuf,
    _config: Option<PathBuf>,
    tolerance: String,
    output: Option<PathBuf>,
    format: String,
    layer_height: f64,
    first_layer_height: f64,
    perimeters: u32,
    infill_density: u32,
    pass_threshold: f64,
    compare_only: bool,
    generated: Option<PathBuf>,
) -> Result<()> {
    info!("Validating G-code output");
    info!("  Input STL: {}", input.display());
    info!("  Reference G-code: {}", reference.display());

    // Parse tolerance level
    let mut validation_config = match tolerance.to_lowercase().as_str() {
        "strict" => ValidationConfig::strict(),
        "relaxed" => ValidationConfig::relaxed(),
        _ => ValidationConfig::default(),
    };
    validation_config.pass_threshold = pass_threshold;

    info!("  Tolerance level: {}", tolerance);
    info!("  Pass threshold: {:.1}", pass_threshold);

    // Parse output format
    let report_format = ReportFormat::from_str(&format).unwrap_or(ReportFormat::Text);

    // Load reference G-code
    info!("Loading reference G-code...");
    let reference_gcode = fs::read_to_string(&reference)
        .with_context(|| format!("Failed to read reference G-code: {}", reference.display()))?;
    let ref_parsed = ParsedGCode::from_string(&reference_gcode);

    info!(
        "  Reference: {} layers, {:.2}mm extrusion, {} moves",
        ref_parsed.layer_count(),
        ref_parsed.total_extrusion,
        ref_parsed.total_moves
    );

    // Get generated G-code
    let generated_gcode = if compare_only {
        // Load pre-generated G-code
        let gen_path = generated.ok_or_else(|| {
            anyhow::anyhow!("--compare-only requires --generated <path> to be specified")
        })?;
        info!("Loading generated G-code from: {}", gen_path.display());
        fs::read_to_string(&gen_path)
            .with_context(|| format!("Failed to read generated G-code: {}", gen_path.display()))?
    } else {
        // Slice the STL to generate G-code
        info!("Slicing STL to generate G-code...");

        let mesh = load_stl(&input).context("Failed to load STL file")?;

        info!("  Mesh loaded: {} triangles", mesh.triangle_count());

        // Configure pipeline to match reference as closely as possible
        let pipeline_config = PipelineConfig::new()
            .layer_height(layer_height)
            .first_layer_height(first_layer_height)
            .perimeters(perimeters)
            .infill_density(infill_density as f64 / 100.0);

        let mut pipeline = PrintPipeline::new(pipeline_config);

        let gcode = pipeline
            .process(&mesh)
            .context("Failed to generate G-code")?;

        info!(
            "  Generated: {} layers, {:.2}mm extrusion",
            gcode.stats.layer_count, gcode.stats.filament_used_mm
        );

        gcode.content().to_string()
    };

    let gen_parsed = ParsedGCode::from_string(&generated_gcode);

    info!(
        "  Generated: {} layers, {:.2}mm extrusion, {} moves",
        gen_parsed.layer_count(),
        gen_parsed.total_extrusion,
        gen_parsed.total_moves
    );

    // Run validation
    info!("Running validation comparison...");
    let report = ValidationReport::generate(&ref_parsed, &gen_parsed, validation_config);

    // Output report
    let report_content = match report_format {
        ReportFormat::Text => report.to_text(),
        ReportFormat::Json => report
            .to_json()
            .context("Failed to serialize report to JSON")?,
        ReportFormat::Html => report.to_html(),
    };

    if let Some(output_path) = output {
        // Write to file
        let final_path = if output_path.extension().is_none() {
            output_path.with_extension(report_format.extension())
        } else {
            output_path
        };

        fs::write(&final_path, &report_content)
            .with_context(|| format!("Failed to write report to: {}", final_path.display()))?;

        println!("Report written to: {}", final_path.display());
    } else {
        // Print to stdout
        println!("{}", report_content);
    }

    // Print summary
    println!();
    println!("═══════════════════════════════════════════════════════════════════");
    if report.passed() {
        println!(
            "✓ VALIDATION PASSED - Quality Score: {:.1}/100",
            report.quality_score()
        );
    } else {
        println!(
            "✗ VALIDATION FAILED - Quality Score: {:.1}/100 (threshold: {:.1})",
            report.quality_score(),
            report.config.pass_threshold
        );
    }
    println!("═══════════════════════════════════════════════════════════════════");

    let (critical, error, warning, info_count) = report.issue_counts();
    if critical > 0 || error > 0 {
        println!(
            "Issues: {} critical, {} errors, {} warnings, {} info",
            critical, error, warning, info_count
        );
    }

    // Return error if validation failed
    if !report.passed() {
        std::process::exit(1);
    }

    Ok(())
}

fn cmd_info(input: PathBuf) -> Result<()> {
    info!("Loading STL file: {}", input.display());

    let mesh = load_stl(&input).context("Failed to load STL file")?;

    let bb = mesh.compute_bounding_box();
    let size_x = bb.max.x - bb.min.x;
    let size_y = bb.max.y - bb.min.y;
    let size_z = bb.max.z - bb.min.z;

    println!("Mesh Information:");
    println!("  File: {}", input.display());
    println!("  Triangles: {}", mesh.triangle_count());
    println!("  Bounding box:");
    println!(
        "    Min: ({:.3}, {:.3}, {:.3}) mm",
        bb.min.x, bb.min.y, bb.min.z
    );
    println!(
        "    Max: ({:.3}, {:.3}, {:.3}) mm",
        bb.max.x, bb.max.y, bb.max.z
    );
    println!("    Size: {:.3} x {:.3} x {:.3} mm", size_x, size_y, size_z);
    println!("  Estimated volume: {:.2} mm³", mesh.volume());
    println!("  Estimated surface area: {:.2} mm²", mesh.surface_area());

    // Estimate layer count at common layer heights
    println!("  Estimated layers:");
    for lh in [0.1, 0.2, 0.3] {
        let layers = (size_z / lh).ceil() as u32;
        println!("    At {:.1}mm layer height: {} layers", lh, layers);
    }

    Ok(())
}

#[cfg(feature = "mcp")]
fn cmd_serve(port: u16, host: String) -> Result<()> {
    info!("Starting MCP server on {}:{}", host, port);

    // TODO: Implement MCP server
    println!("MCP server not yet implemented");
    println!("Use --help for available commands");

    Ok(())
}

fn cmd_list_printers(data_dir: Option<PathBuf>, brand_filter: Option<String>) -> Result<()> {
    let data_path = data_dir.unwrap_or_else(get_default_data_dir);
    info!("Loading profiles from: {}", data_path.display());

    let registry =
        ProfileRegistry::load_from_directory(&data_path).context("Failed to load profiles")?;

    println!("Available Printers ({} total):", registry.printer_count());
    println!("{:-<60}", "");

    let mut printers: Vec<_> = registry.printer_ids().collect();
    printers.sort();

    for id in printers {
        if let Some(printer) = registry.get_printer(id) {
            // Apply brand filter if specified
            if let Some(ref brand) = brand_filter {
                if !printer.brand.to_lowercase().contains(&brand.to_lowercase()) {
                    continue;
                }
            }

            let nozzle_str = format!("{:.1}mm", printer.default_nozzle_diameter);

            println!(
                "  {:<35} {:>8}  {}x{}x{}mm",
                id,
                nozzle_str,
                printer.build_volume.x as i32,
                printer.build_volume.y as i32,
                printer.build_volume.z as i32
            );
        }
    }

    println!();
    println!("Use --printer <ID> with the slice command");

    Ok(())
}

fn cmd_list_filaments(
    data_dir: Option<PathBuf>,
    material_filter: Option<String>,
    brand_filter: Option<String>,
) -> Result<()> {
    let data_path = data_dir.unwrap_or_else(get_default_data_dir);
    info!("Loading profiles from: {}", data_path.display());

    let registry =
        ProfileRegistry::load_from_directory(&data_path).context("Failed to load profiles")?;

    println!("Available Filaments ({} total):", registry.filament_count());
    println!("{:-<60}", "");

    let mut filaments: Vec<_> = registry.filament_ids().collect();
    filaments.sort();

    for id in filaments {
        if let Some(filament) = registry.get_filament(id) {
            // Apply material filter if specified
            if let Some(ref material) = material_filter {
                if !filament
                    .material
                    .to_lowercase()
                    .contains(&material.to_lowercase())
                {
                    continue;
                }
            }

            // Apply brand filter if specified
            if let Some(ref brand) = brand_filter {
                if !filament
                    .brand
                    .to_lowercase()
                    .contains(&brand.to_lowercase())
                {
                    continue;
                }
            }

            let temp_str = format!("{}°C", filament.temperatures.nozzle.default as i32);

            println!("  {:<35} {:>8}  {}", id, filament.material, temp_str);
        }
    }

    println!();
    println!("Use --filament <ID> with the slice command");

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn verify_cli() {
        use clap::CommandFactory;
        Cli::command().debug_assert();
    }
}
