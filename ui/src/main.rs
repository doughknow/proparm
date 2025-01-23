#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")] // hide console window on Windows in release

use eframe::egui;
use egui_plot::{Line, Plot, PlotPoints};
use std::{sync::mpsc, thread, time};

#[derive(PartialEq)]
enum Menu {
    Filters,
    Algos,
}

#[derive(PartialEq)]
enum Filter {
    Complementary,
    Kalman,
}

#[derive(PartialEq)]
enum Controller {
    PID,
    Cascade,
    LQR
}

struct MyApp {
    tx: mpsc::Sender<String>,
    rx: mpsc::Receiver<String>,
    kp: f64,
    ki: f64,
    kd: f64,
    a: f64,
    x: f64,
    y: f64,
    z: f64,
    page: Menu,
    filter: Filter,
    controller: Controller,
    data: Vec<(f64, f64)>, // Store (x, y) pairs for the plot
    time: f64,             // A variable to simulate time for updating the chart
}

impl MyApp {
    fn new(tx: mpsc::Sender<String>, rx: mpsc::Receiver<String>) -> Self {
        Self {
            tx,
            rx,
            kp: 0.5,
            ki: 0.5,
            kd: 0.5,
            a: 0.99,
            x: 1.,
            y: 2.,
            z: 3.,
            page: Menu::Filters,
            filter: Filter::Complementary,
            controller: Controller::PID,
            data: Vec::new(),
            time: 0.,
        }
    }
}

impl eframe::App for MyApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        egui::TopBottomPanel::top("top_panel").show(ctx, |ui| {
            egui::menu::bar(ui, |ui| {
                if ui.button("Filtres").clicked() {
                    self.page = Menu::Filters;
                }
                if ui.button("Algorithmes").clicked() {
                    self.page = Menu::Algos;
                }
            });
        });

        egui::CentralPanel::default().show(ctx, |ui| {
            ui.vertical(|ui| {
                ui.group(|ui| {
                    if self.page == Menu::Filters {
                        ui.horizontal(|ui| {
                            ui.heading("Filtre complémentaire");
                            if self.filter != Filter::Complementary
                                && ui.button("Activer").clicked()
                            {
                                self.filter = Filter::Complementary;
                                self.tx.send("k".to_string()).unwrap();
                            }
                        });
                        ui.horizontal(|ui| {
                            ui.label("Alpha");
                            let t = self.a;
                            ui.add(egui::Slider::new(&mut self.a, 0.0..=1.0));
                            if t != self.a {
                                self.tx.send(format!("a:{}", self.a)).unwrap();
                            }
                        });

                        ui.separator();

                        ui.horizontal(|ui| {
                            ui.heading("Filtre Kalman");
                            if self.filter != Filter::Kalman && ui.button("Activer").clicked() {
                                self.filter = Filter::Kalman;
                                self.tx.send("K".to_string()).unwrap();
                            }
                        });
                        ui.horizontal(|ui| {
                            ui.label("x");
                            let t = self.x;
                            ui.add(egui::Slider::new(&mut self.x, 0.0..=1.0));
                            if t != self.x {
                                self.tx.send(format!("x:{}", self.x)).unwrap();
                            }
                        });
                        ui.horizontal(|ui| {
                            ui.label("y");
                            let t = self.y;
                            ui.add(egui::Slider::new(&mut self.y, 0.0..=1.0));
                            if t != self.y {
                                self.tx.send(format!("y:{}", self.y)).unwrap();
                            }
                        });
                        ui.horizontal(|ui| {
                            ui.label("z");
                            let t = self.z;
                            ui.add(egui::Slider::new(&mut self.z, 0.0..=1.0));
                            if t != self.z {
                                self.tx.send(format!("z:{}", self.z)).unwrap();
                            }
                        });
                    }

                    if self.page == Menu::Algos {
                        ui.horizontal(|ui| {
                            ui.heading("PID");
                            if self.controller != Controller::PID && ui.button("Activer").clicked()
                            {
                                self.controller = Controller::PID;
                                self.tx.send("P".to_string()).unwrap();
                            }
                        });
                        ui.horizontal(|ui| {
                            ui.label("P");
                            let t = self.kp;
                            ui.add(egui::Slider::new(&mut self.kp, 0.0..=1.0));
                            if t != self.kp {
                                self.tx.send(format!("p:{}", self.kp)).unwrap();
                            }
                        });
                        ui.horizontal(|ui| {
                            ui.label("I");
                            let t = self.ki;
                            ui.add(egui::Slider::new(&mut self.ki, 0.0..=1.0));
                            if t != self.ki {
                                self.tx.send(format!("i:{}", self.ki)).unwrap();
                            }
                        });
                        ui.horizontal(|ui| {
                            ui.label("D");
                            let t = self.kd;
                            ui.add(egui::Slider::new(&mut self.kd, 0.0..=1.0));
                            if t != self.kd {
                                self.tx.send(format!("d:{}", self.kd)).unwrap();
                            }
                        });

                        ui.separator();

                        ui.horizontal(|ui| {
                            ui.heading("Cascade");
                            if self.controller != Controller::Cascade
                                && ui.button("Activer").clicked()
                            {
                                self.controller = Controller::Cascade;
                            }
                        });
                        ui.horizontal(|ui| {
                            ui.label("Alpha");
                            ui.add(egui::Slider::new(&mut self.a, 0.0..=1.0));
                        });

                        ui.separator();

                        ui.horizontal(|ui| {
                            ui.heading("LQR");
                            if self.controller != Controller::LQR
                                && ui.button("Activer").clicked()
                            {
                                self.controller = Controller::LQR;
                            }
                        });
                        ui.horizontal(|ui| {
                            ui.label("a");
                            ui.add(egui::Slider::new(&mut self.a, 0.0..=10.0));
                        });
                        ui.horizontal(|ui| {
                            ui.label("b");
                            ui.add(egui::Slider::new(&mut self.a, 0.0..=10.0));
                        });
                    }
                });

                ui.group(|ui| {
                    ui.label("Angle");
                    let sin: PlotPoints = self
                        .data
                        .iter()
                        .map(|(x, y)| [x.clone(), y.clone()])
                        .collect();
                    let line = Line::new(sin);
                    Plot::new("angle")
                        .view_aspect(3.0)
                        .show(ui, |plot_ui| plot_ui.line(line));
                });
            });
        });

        if let Ok(v) = self.rx.try_recv() {
            self.time += 0.1;

            self.data.push((
                self.time,
                v.trim()
                    .chars()
                    .filter(|&c| !c.is_ascii_control()) // Filter out control characters like '\0', '\n', etc.
                    .collect::<String>()
                    .parse()
                    .expect("Failed to parse the string to f64"),
            ));

            // Keep the data size under a certain limit to avoid excessive memory usage
            if self.data.len() > 100 {
                self.data.remove(0); // Remove the oldest data point
            }
        }
    }
}

fn main() {
    let (tx_app_to_mcu, rx_app_to_mcu) = mpsc::channel::<String>();
    let (tx_mcu_to_app, rx_mcu_to_app) = mpsc::channel::<String>();

    let handle_a = thread::spawn(move || {
        let port_name = "/dev/ttyUSB0";
        let baud_rate = 115200;

        let mut port = serialport::new(port_name, baud_rate).open().unwrap();

        loop {
            let mut buffer = [0; 64];
            let n = port.read(&mut buffer[..]);
            if let Ok(t) = n {
                if t != 0 {
                    if let Ok(s) = std::str::from_utf8(&buffer) {
                        // println!("{:?}", s);
                        if let Some(pos) = s.find('\n') {
                            let data = &s[..pos];
                            // println!("rx: {:?}", data);
                            tx_mcu_to_app.send(data.to_string()).unwrap();
                        }
                    } else {
                        println!("Invalid UTF-8 sequence");
                    }
                }
            }

            if let Ok(v) = rx_app_to_mcu.try_recv() {
                port.write_all(v.as_bytes()).ok();
                println!("sending...");
            }

            thread::sleep(time::Duration::from_millis(10));
        }
    });

    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default().with_inner_size([320.0, 240.0]),
        
        ..Default::default()
    };
    eframe::run_native(
        "Proparm",
        options,
        Box::new(|_| Ok(Box::new(MyApp::new(tx_app_to_mcu, rx_mcu_to_app)))),
    )
    .unwrap();

    handle_a.join().unwrap();
}
