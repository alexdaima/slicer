import * as React from "react"
import { Slot } from "@radix-ui/react-slot"
import { cva, type VariantProps } from "class-variance-authority"

import { cn } from "@/lib/utils"

const buttonVariants = cva(
  "inline-flex items-center justify-center whitespace-nowrap rounded-md text-sm font-medium transition-all focus-visible:outline-none focus-visible:ring-1 focus-visible:ring-[#a78bfa] disabled:pointer-events-none disabled:opacity-50 border-none",
  {
    variants: {
      variant: {
        default: "!bg-[#a78bfa] !text-[#1a1a1a] hover:shadow-[0_0_20px_rgba(167,139,250,0.3)] font-semibold",
        destructive:
          "!bg-[#ef4444] !text-white hover:!bg-[#dc2626]",
        outline:
          "border !border-[#333333] !bg-transparent hover:!bg-[#2a2a2a] hover:!text-white !text-[#a0a0b0]",
        secondary:
          "!bg-[#252525] !text-[#a0a0b0] hover:!bg-[#2a2a2a] hover:!text-white",
        ghost: "!bg-transparent hover:!bg-[#2a2a2a] hover:!text-white !text-[#a0a0b0]",
        link: "!text-[#a78bfa] underline-offset-4 hover:underline !bg-transparent",
      },
      size: {
        default: "h-10 px-4 py-2",
        sm: "h-8 px-3 text-xs",
        lg: "h-11 px-8",
        icon: "h-10 w-10",
      },
    },
    defaultVariants: {
      variant: "default",
      size: "default",
    },
  }
)

export interface ButtonProps
  extends React.ButtonHTMLAttributes<HTMLButtonElement>,
    VariantProps<typeof buttonVariants> {
  asChild?: boolean
}

const Button = React.forwardRef<HTMLButtonElement, ButtonProps>(
  ({ className, variant, size, asChild = false, ...props }, ref) => {
    const Comp = asChild ? Slot : "button"
    return (
      <Comp
        className={cn(buttonVariants({ variant, size, className }))}
        ref={ref}
        {...props}
      />
    )
  }
)
Button.displayName = "Button"

export { Button, buttonVariants }
