﻿#pragma checksum "..\..\CalcIK_2D_AllTwoBoneAnalytic.xaml" "{406ea660-64cf-4c82-b6f0-42d48172a799}" "E547FA9C8A19EC6D018FC99DA04D4AF0"
//------------------------------------------------------------------------------
// <auto-generated>
//     Этот код создан программой.
//     Исполняемая версия:4.0.30319.42000
//
//     Изменения в этом файле могут привести к неправильной работе и будут потеряны в случае
//     повторной генерации кода.
// </auto-generated>
//------------------------------------------------------------------------------

using System;
using System.Diagnostics;
using System.Windows;
using System.Windows.Automation;
using System.Windows.Controls;
using System.Windows.Controls.Primitives;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Ink;
using System.Windows.Input;
using System.Windows.Markup;
using System.Windows.Media;
using System.Windows.Media.Animation;
using System.Windows.Media.Effects;
using System.Windows.Media.Imaging;
using System.Windows.Media.Media3D;
using System.Windows.Media.TextFormatting;
using System.Windows.Navigation;
using System.Windows.Shapes;


namespace RJ_Demo_IK {
    
    
    /// <summary>
    /// CalcIK_2D_AllTwoBoneAnalytic
    /// </summary>
    public partial class CalcIK_2D_AllTwoBoneAnalytic : System.Windows.Controls.UserControl, System.Windows.Markup.IComponentConnector {
        
        
        #line 29 "..\..\CalcIK_2D_AllTwoBoneAnalytic.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal RJ_Demo_IK.CalcIK_2D_AllTwoBoneAnalytic _thisWindow;
        
        #line default
        #line hidden
        
        
        #line 30 "..\..\CalcIK_2D_AllTwoBoneAnalytic.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.Grid _mainGrid;
        
        #line default
        #line hidden
        
        
        #line 33 "..\..\CalcIK_2D_AllTwoBoneAnalytic.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.ColumnDefinition _viewportColumn;
        
        #line default
        #line hidden
        
        
        #line 72 "..\..\CalcIK_2D_AllTwoBoneAnalytic.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.Canvas _viewport;
        
        #line default
        #line hidden
        
        
        #line 76 "..\..\CalcIK_2D_AllTwoBoneAnalytic.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Shapes.Line _solution1_bone1_line;
        
        #line default
        #line hidden
        
        
        #line 84 "..\..\CalcIK_2D_AllTwoBoneAnalytic.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Shapes.Line _solution1_bone2_line;
        
        #line default
        #line hidden
        
        
        #line 92 "..\..\CalcIK_2D_AllTwoBoneAnalytic.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Shapes.Line _solution1_bone1_extended_line;
        
        #line default
        #line hidden
        
        
        #line 100 "..\..\CalcIK_2D_AllTwoBoneAnalytic.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Shapes.Line _solution2_bone1_line;
        
        #line default
        #line hidden
        
        
        #line 108 "..\..\CalcIK_2D_AllTwoBoneAnalytic.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Shapes.Line _solution2_bone2_line;
        
        #line default
        #line hidden
        
        
        #line 116 "..\..\CalcIK_2D_AllTwoBoneAnalytic.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Shapes.Line _solution2_bone1_extended_line;
        
        #line default
        #line hidden
        
        
        #line 124 "..\..\CalcIK_2D_AllTwoBoneAnalytic.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Shapes.Ellipse _maxRangeEllipse;
        
        #line default
        #line hidden
        
        
        #line 131 "..\..\CalcIK_2D_AllTwoBoneAnalytic.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Shapes.Ellipse _minRangeEllipse;
        
        #line default
        #line hidden
        
        
        #line 138 "..\..\CalcIK_2D_AllTwoBoneAnalytic.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Shapes.Ellipse _solution1_bone2RangeEllipse;
        
        #line default
        #line hidden
        
        
        #line 146 "..\..\CalcIK_2D_AllTwoBoneAnalytic.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Shapes.Ellipse _solution2_bone2RangeEllipse;
        
        #line default
        #line hidden
        
        
        #line 154 "..\..\CalcIK_2D_AllTwoBoneAnalytic.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Shapes.Ellipse _targetEllipse;
        
        #line default
        #line hidden
        
        
        #line 162 "..\..\CalcIK_2D_AllTwoBoneAnalytic.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Shapes.Line _xAxisLine;
        
        #line default
        #line hidden
        
        
        #line 163 "..\..\CalcIK_2D_AllTwoBoneAnalytic.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Shapes.Line _yAxisLine;
        
        #line default
        #line hidden
        
        
        #line 164 "..\..\CalcIK_2D_AllTwoBoneAnalytic.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Shapes.Path _solution1_theta1ArcPath;
        
        #line default
        #line hidden
        
        
        #line 182 "..\..\CalcIK_2D_AllTwoBoneAnalytic.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Shapes.Path _solution1_theta2ArcPath;
        
        #line default
        #line hidden
        
        
        #line 200 "..\..\CalcIK_2D_AllTwoBoneAnalytic.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Shapes.Path _solution2_theta1ArcPath;
        
        #line default
        #line hidden
        
        
        #line 218 "..\..\CalcIK_2D_AllTwoBoneAnalytic.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Shapes.Path _solution2_theta2ArcPath;
        
        #line default
        #line hidden
        
        
        #line 243 "..\..\CalcIK_2D_AllTwoBoneAnalytic.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Documents.Hyperlink _websiteLink;
        
        #line default
        #line hidden
        
        private bool _contentLoaded;
        
        /// <summary>
        /// InitializeComponent
        /// </summary>
        [System.Diagnostics.DebuggerNonUserCodeAttribute()]
        [System.CodeDom.Compiler.GeneratedCodeAttribute("PresentationBuildTasks", "4.0.0.0")]
        public void InitializeComponent() {
            if (_contentLoaded) {
                return;
            }
            _contentLoaded = true;
            System.Uri resourceLocater = new System.Uri("/RJ_Demo_IK;component/calcik_2d_alltwoboneanalytic.xaml", System.UriKind.Relative);
            
            #line 1 "..\..\CalcIK_2D_AllTwoBoneAnalytic.xaml"
            System.Windows.Application.LoadComponent(this, resourceLocater);
            
            #line default
            #line hidden
        }
        
        [System.Diagnostics.DebuggerNonUserCodeAttribute()]
        [System.CodeDom.Compiler.GeneratedCodeAttribute("PresentationBuildTasks", "4.0.0.0")]
        [System.ComponentModel.EditorBrowsableAttribute(System.ComponentModel.EditorBrowsableState.Never)]
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Design", "CA1033:InterfaceMethodsShouldBeCallableByChildTypes")]
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Maintainability", "CA1502:AvoidExcessiveComplexity")]
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1800:DoNotCastUnnecessarily")]
        void System.Windows.Markup.IComponentConnector.Connect(int connectionId, object target) {
            switch (connectionId)
            {
            case 1:
            this._thisWindow = ((RJ_Demo_IK.CalcIK_2D_AllTwoBoneAnalytic)(target));
            
            #line 29 "..\..\CalcIK_2D_AllTwoBoneAnalytic.xaml"
            this._thisWindow.SizeChanged += new System.Windows.SizeChangedEventHandler(this._thisWindow_SizeChanged);
            
            #line default
            #line hidden
            return;
            case 2:
            this._mainGrid = ((System.Windows.Controls.Grid)(target));
            return;
            case 3:
            this._viewportColumn = ((System.Windows.Controls.ColumnDefinition)(target));
            return;
            case 4:
            this._viewport = ((System.Windows.Controls.Canvas)(target));
            
            #line 73 "..\..\CalcIK_2D_AllTwoBoneAnalytic.xaml"
            this._viewport.MouseLeftButtonDown += new System.Windows.Input.MouseButtonEventHandler(this.viewport_MouseLeftButtonDown);
            
            #line default
            #line hidden
            
            #line 74 "..\..\CalcIK_2D_AllTwoBoneAnalytic.xaml"
            this._viewport.MouseLeftButtonUp += new System.Windows.Input.MouseButtonEventHandler(this.viewport_MouseLeftButtonUp);
            
            #line default
            #line hidden
            
            #line 75 "..\..\CalcIK_2D_AllTwoBoneAnalytic.xaml"
            this._viewport.MouseMove += new System.Windows.Input.MouseEventHandler(this.viewport_MouseMove);
            
            #line default
            #line hidden
            return;
            case 5:
            this._solution1_bone1_line = ((System.Windows.Shapes.Line)(target));
            return;
            case 6:
            this._solution1_bone2_line = ((System.Windows.Shapes.Line)(target));
            return;
            case 7:
            this._solution1_bone1_extended_line = ((System.Windows.Shapes.Line)(target));
            return;
            case 8:
            this._solution2_bone1_line = ((System.Windows.Shapes.Line)(target));
            return;
            case 9:
            this._solution2_bone2_line = ((System.Windows.Shapes.Line)(target));
            return;
            case 10:
            this._solution2_bone1_extended_line = ((System.Windows.Shapes.Line)(target));
            return;
            case 11:
            this._maxRangeEllipse = ((System.Windows.Shapes.Ellipse)(target));
            return;
            case 12:
            this._minRangeEllipse = ((System.Windows.Shapes.Ellipse)(target));
            return;
            case 13:
            this._solution1_bone2RangeEllipse = ((System.Windows.Shapes.Ellipse)(target));
            return;
            case 14:
            this._solution2_bone2RangeEllipse = ((System.Windows.Shapes.Ellipse)(target));
            return;
            case 15:
            this._targetEllipse = ((System.Windows.Shapes.Ellipse)(target));
            return;
            case 16:
            this._xAxisLine = ((System.Windows.Shapes.Line)(target));
            return;
            case 17:
            this._yAxisLine = ((System.Windows.Shapes.Line)(target));
            return;
            case 18:
            this._solution1_theta1ArcPath = ((System.Windows.Shapes.Path)(target));
            return;
            case 19:
            this._solution1_theta2ArcPath = ((System.Windows.Shapes.Path)(target));
            return;
            case 20:
            this._solution2_theta1ArcPath = ((System.Windows.Shapes.Path)(target));
            return;
            case 21:
            this._solution2_theta2ArcPath = ((System.Windows.Shapes.Path)(target));
            return;
            case 22:
            this._websiteLink = ((System.Windows.Documents.Hyperlink)(target));
            
            #line 243 "..\..\CalcIK_2D_AllTwoBoneAnalytic.xaml"
            this._websiteLink.Click += new System.Windows.RoutedEventHandler(this._websiteLink_Click);
            
            #line default
            #line hidden
            return;
            }
            this._contentLoaded = true;
        }
    }
}
