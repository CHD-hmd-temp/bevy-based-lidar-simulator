#![allow(unused)]
use std::io;
use std::io::BufRead;

pub trait FromInput: Sized {
    fn from_input(input: &str) -> Result<Self, &'static str>;
}

#[derive(PartialEq, Eq)]
pub enum InputHandling {
    Ignore,
    Accept,
}

#[allow(unused)]
pub trait DefaultHandling {
    fn handle_default() -> InputHandling;
}

impl DefaultHandling for f32 {
    fn handle_default() -> InputHandling {
        InputHandling::Accept
    }
}

impl DefaultHandling for String {
    fn handle_default() -> InputHandling {
        InputHandling::Ignore
    }
}

impl DefaultHandling for u32 {
    fn handle_default() -> InputHandling {
        InputHandling::Accept
    }
}

impl DefaultHandling for char {
    fn handle_default() -> InputHandling {
        InputHandling::Accept
    }
}

pub fn read_with_default<T>(prompt: &str, default: T, input_source: Option<&mut dyn BufRead>) -> T
where
    T: FromInput + ToString + DefaultHandling,
{
    println!("{}", prompt);

    let mut input = String::new();
    let read_result = if let Some(source) = input_source {
        source.read_line(&mut input)
    } else {
        io::stdin().read_line(&mut input)
    };

    if read_result.is_err() {
        println!("Error reading input. Using default: {}", default.to_string());
        return default;
    }

    let trimmed = input.trim();
    
    if trimmed.is_empty() {
        println!("Empty input. Using default: {}", default.to_string());
        return default;
    }

    match T::from_input(trimmed) {
        Ok(value) => value,
        Err(err) => {
            println!("{} Using default: {}", err, default.to_string());
            default
        }
    }
}

impl FromInput for f32 {
    fn from_input(input: &str) -> Result<Self, &'static str> {
        input
            .parse()
            .map_err(|_| "Invalid float value")
    }
}

impl FromInput for String {
    fn from_input(input: &str) -> Result<Self, &'static str> {
        Ok(input.to_string())
    }
}

impl FromInput for u32 {
    fn from_input(input: &str) -> Result<Self, &'static str> {
        input
            .parse()
            .map_err(|_| "Invalid integer value")
    }
}

impl FromInput for char {
    fn from_input(input: &str) -> Result<Self, &'static str> {
        input
            .chars()
            .next()
            .ok_or("Invalid char value")
    }
}

// impl From<String> for String {
//     fn from(value: String) -> Self {
//         value
//     }
// }