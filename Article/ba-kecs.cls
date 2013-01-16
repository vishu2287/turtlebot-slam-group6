\NeedsTeXFormat{LaTeX2e} 
\ProvidesClass{ba-kecs}[2005/01/04]
\typeout{Conference Style, Knowledge Engineering Bachelor}
% -----------------------
\LoadClass[a4paper,10pt,twoside]{article}
\RequirePackage{fancyhdr}
\RequirePackage[dutch,english]{babel}

\newcommand{\lang}{english}
\DeclareOption{dutch}{
  \renewcommand{\lang}{dutch}
%  \renewcommand{\abstractname}{Samenvatting}
}
\ProcessOptions

% Physical page layout
\twocolumn \flushbottom \sloppy
% Note that TeX has built-in 1-inch top and left margins.
\setlength\topmargin{-0.25in} \setlength\oddsidemargin{-0.25in}
\setlength\evensidemargin{-0.25in} \setlength\textheight{9.0in}
\setlength\textwidth{7.0in} \setlength\columnsep{0.25in}
\newlength\titlebox \setlength\titlebox{2.25in}

\def\maketitle{\par
 \select@language{\lang}
 \typeout{\lang}
 \twocolumn[\@maketitle] \@thanks
 \let\maketitle\relax \let\@maketitle\relax
}




% Sections with less space
\def\section{\@startsection{section}{1}{\z@}{-10pt plus
    -3pt minus -2pt}{4pt plus 2pt minus 1pt}{\Large\bf\raggedright}}
\def\subsection{\@startsection{subsection}{2}{\z@}{-8pt plus
    -2pt minus -2pt}{3pt plus 2pt minus 1pt}{\large\bf\raggedright}}
\def\subsubsection{\@startsection{subparagraph}{3}{\z@}{-6pt plus
   -2pt minus -1pt}{1pt plus 1pt minus 1pt}{\normalsize\bf\raggedright}}
\setcounter{secnumdepth}{2} % Don't number subsubsections

% Footnotes
\footnotesep 6.65pt \skip\footins 9pt plus 4pt minus 2pt
\def\footnoterule{\kern-3pt \hrule width 5pc \kern 2.6pt }
\setcounter{footnote}{0}

% Illustrations (floats)
\floatsep 12pt plus 2pt minus 2pt \textfloatsep 16pt plus 2pt
minus 4pt \intextsep 12pt plus 2pt minus 2pt \dblfloatsep 12pt
plus 2pt minus 2pt \dbltextfloatsep 18pt plus 2pt minus 4pt

% Displays
\abovedisplayskip 7pt plus2pt minus5pt%
\belowdisplayskip \abovedisplayskip
\abovedisplayshortskip  0pt plus3pt%
\belowdisplayshortskip  4pt plus3pt minus3pt%

% Lists
\leftmargini 2em \leftmarginii 2em \leftmarginiii 1em
\leftmarginiv 0.5em \leftmarginv 0.5em \leftmarginvi 0.5em

\leftmargin\leftmargini \labelsep 5pt
\labelwidth\leftmargini\advance\labelwidth-\labelsep

\def\@listI{\leftmargin\leftmargini
\parsep 2pt plus 1pt minus 0.5pt%
\topsep 4pt plus 1pt minus 2pt%
\itemsep 2pt plus 1pt minus 0.5pt%
\partopsep 1pt plus 0.5pt minus 0.5pt}

\let\@listi\@listI
\@listi

\def\@listii{\leftmargin\leftmarginii
   \labelwidth\leftmarginii\advance\labelwidth-\labelsep
   \parsep 1pt plus 0.5pt minus 0.5pt
   \topsep 2pt plus 1pt minus 0.5pt
   \itemsep \parsep}
\def\@listiii{\leftmargin\leftmarginiii
    \labelwidth\leftmarginiii\advance\labelwidth-\labelsep
    \parsep 0pt plus 1pt
    \partopsep 0.5pt plus 0pt minus 0.5pt
    \topsep 1pt plus 0.5pt minus 0.5pt
    \itemsep \topsep}
\def\@listiv{\leftmargin\leftmarginiv
     \labelwidth\leftmarginiv\advance\labelwidth-\labelsep}
\def\@listv{\leftmargin\leftmarginv
     \labelwidth\leftmarginv\advance\labelwidth-\labelsep}
\def\@listvi{\leftmargin\leftmarginvi
     \labelwidth\leftmarginvi\advance\labelwidth-\labelsep}

\renewenvironment{abstract}{\centerline{\Large\bf
\abstractname}\vspace{0.5ex}\begin{quote}}{\par\end{quote}\vskip 1ex}

\newcommand{\TheAuthor}{}
\renewcommand{\author}[1]{\gdef\@author{#1}\renewcommand{\TheAuthor}{#1}}
\newcommand{\TheRunningTitle}{}
\newcommand{\runningtitle}[1]{\gdef\@runtitle{#1}\renewcommand{\TheRunningTitle}{#1}}

\fancyhf{}
\fancyfoot[C]{\footnotesize (v. \today, p.\thepage)}
\fancyhead[RO,LE]{\footnotesize \TheAuthor}
\fancyhead[LO,RE]{\footnotesize \TheRunningTitle}
\renewcommand{\sectionmark}[1]{}
\renewcommand{\subsectionmark}[1]{}
\renewcommand\headrulewidth{0pt}
\renewcommand\footrulewidth{0pt}
\pagestyle{fancy}
\thispagestyle{empty}
\bibliographystyle{ba-kecs}

\newcommand{\url}[1]{{\tt #1}}
