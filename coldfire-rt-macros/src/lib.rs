//! Attributes for coldfire-rt. `#[entry]` marks the reset handler's `main`; `#[exception]` and `#[interrupt]`
//! turn `fn NAME()` into a register-saving trampoline (coldfire-rt's `exception_entry!`) and a
//! `coldfire_rt::Handler` record in `.coldfire_rt.handlers`, which startup copies into the RAM vector table.
use proc_macro::TokenStream;
use proc_macro2::{Span, TokenStream as T2};
use quote::{format_ident, quote};
use syn::parse::Parser;
use syn::{ItemFn, Path, ReturnType, Type, parse_macro_input};

/// CPU exception vectors below 64 that `#[exception]` knows by name (ColdFire exception vector table).
const EXCEPTIONS: &[(&str, usize)] = &[
    ("AccessError", 2),
    ("AddressError", 3),
    ("IllegalInstruction", 4),
    ("DivideByZero", 5),
    ("PrivilegeViolation", 8),
    ("Trace", 9),
    ("UnimplementedLineA", 10),
    ("UnimplementedLineF", 11),
    ("FormatError", 14),
    ("SpuriousInterrupt", 24),
];

/// Trap0..Trap15 (no leading zeros) → 32..47, else the named table.
fn exception_vector(name: &str) -> Option<usize> {
    if let Some(n) = name.strip_prefix("Trap") {
        let ok = !n.is_empty() && n.bytes().all(|b| b.is_ascii_digit()) && (n == "0" || !n.starts_with('0'));
        return n.parse::<usize>().ok().filter(|n| ok && *n <= 15).map(|n| 32 + n);
    }
    EXCEPTIONS.iter().find(|(n, _)| *n == name).map(|(_, v)| *v)
}

fn error(span: Span, msg: &str) -> TokenStream {
    syn::Error::new(span, msg).to_compile_error().into()
}

fn plain_handler(f: &ItemFn) -> bool {
    f.sig.inputs.is_empty()
        && matches!(f.sig.output, ReturnType::Default)
        && f.sig.asyncness.is_none()
        && f.sig.generics.params.is_empty()
}

/// The reset handler's `main`: `#[entry] fn main() -> ! { … }`.
#[proc_macro_attribute]
pub fn entry(args: TokenStream, input: TokenStream) -> TokenStream {
    let f = parse_macro_input!(input as ItemFn);
    if !args.is_empty() {
        return error(Span::call_site(), "#[entry] takes no arguments");
    }
    let never = matches!(&f.sig.output, ReturnType::Type(_, t) if matches!(**t, Type::Never(_)));
    if !never || !f.sig.inputs.is_empty() || f.sig.asyncness.is_some() {
        return error(f.sig.ident.span(), "#[entry] must be `fn() -> !`");
    }
    let name = &f.sig.ident;
    quote! {
        use ::coldfire_rt as _;
        #[unsafe(export_name = "main")]
        pub extern "C" fn __coldfire_rt_main() -> ! {
            #name()
        }
        #f
    }
    .into()
}

/// A CPU exception handler, named after its vector: `#[exception] fn Trap0() { … }` (Trap0..Trap15,
/// AccessError, AddressError, IllegalInstruction, DivideByZero, PrivilegeViolation, Trace,
/// UnimplementedLineA, UnimplementedLineF, FormatError, SpuriousInterrupt).
#[proc_macro_attribute]
pub fn exception(args: TokenStream, input: TokenStream) -> TokenStream {
    let f = parse_macro_input!(input as ItemFn);
    if !args.is_empty() {
        return error(Span::call_site(), "#[exception] takes no arguments");
    }
    let Some(n) = exception_vector(&f.sig.ident.to_string()) else {
        return error(f.sig.ident.span(), "unknown exception: Trap0..Trap15, AccessError, AddressError, IllegalInstruction, DivideByZero, PrivilegeViolation, Trace, UnimplementedLineA, UnimplementedLineF, FormatError or SpuriousInterrupt");
    };
    handler(&f, quote!(#n))
}

/// An interrupt handler named after its `Interrupt` variant: `#[interrupt] fn PIT0() { … }` takes the vector
/// from `interrupt::PIT0` in scope (the PAC exports both the attribute and `Interrupt as interrupt`), or
/// from `#[interrupt(vectors = path::to::Interrupt)]`.
#[proc_macro_attribute]
pub fn interrupt(args: TokenStream, input: TokenStream) -> TokenStream {
    let f = parse_macro_input!(input as ItemFn);
    let vectors: Path = if args.is_empty() {
        syn::parse_quote!(interrupt)
    } else {
        let parser = |s: syn::parse::ParseStream| {
            let key: syn::Ident = s.parse()?;
            if key != "vectors" {
                return Err(syn::Error::new(key.span(), "expected `vectors = <path to the Interrupt enum>`"));
            }
            s.parse::<syn::Token![=]>()?;
            s.parse::<Path>()
        };
        match parser.parse(args) {
            Ok(p) => p,
            Err(e) => return e.to_compile_error().into(),
        }
    };
    let id = &f.sig.ident;
    handler(&f, quote!(#vectors::#id as usize))
}

fn handler(f: &ItemFn, vector: T2) -> TokenStream {
    if !plain_handler(f) {
        return error(f.sig.ident.span(), "a handler must be `fn NAME()`");
    }
    let id = &f.sig.ident;
    let body = format_ident!("__coldfire_rt_h_{}", id);
    let entry = format_ident!("__coldfire_rt_e_{}", id);
    let record = format_ident!("__COLDFIRE_RT_V_{}", id);
    let body_sym = body.to_string();
    let attrs = &f.attrs;
    let block = &f.block;
    quote! {
        #(#attrs)*
        #[allow(non_snake_case)]
        #[unsafe(export_name = #body_sym)]
        extern "C" fn #body() #block
        ::coldfire_rt::exception_entry!(#entry, #body);
        const _: () = assert!((#vector) >= 2 && (#vector) < 256, "vector out of range");
        #[unsafe(link_section = ".coldfire_rt.handlers")]
        #[used]
        #[allow(non_upper_case_globals)]
        static #record: ::coldfire_rt::Handler = ::coldfire_rt::Handler { vector: #vector, entry: #entry };
    }
    .into()
}

#[cfg(test)]
mod tests {
    use super::exception_vector;

    #[test]
    fn exceptions_by_name() {
        assert_eq!(exception_vector("Trap0"), Some(32));
        assert_eq!(exception_vector("Trap15"), Some(47));
        assert_eq!(exception_vector("IllegalInstruction"), Some(4));
        assert_eq!(exception_vector("Trap16"), None);
        assert_eq!(exception_vector("Trap01"), None);
        assert_eq!(exception_vector("Trap"), None);
        assert_eq!(exception_vector("PIT0"), None);
    }
}
